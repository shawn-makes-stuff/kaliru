// Support for reading acceleration data from an LIS2DW chip
// (port of klippy/extras/lis2dw.py)
//
// Copyright (C) 2023  Zhou.XianMing <zhouxm@biqu3d.com>
// Copyright (C) 2020-2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::adxl345::{accel_header, batch_msg, convert_sample, read_axes_map, AccelChip, AccelCommandHelper, AccelQueryHelper, AxisMap, BATCH_UPDATES};
use crate::extras::bulk_sensor::{BatchBulkHelper, FixedFreqReader};
use crate::extras::bus::{mcu_i2c_from_config, mcu_spi_from_config, McuI2c, McuSpi};
use crate::logging;
use crate::mcu::{CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

// LIS2DW registers
const REG_LIS2DW_WHO_AM_I_ADDR: i64 = 0x0F;
const REG_LIS2DW_CTRL_REG1_ADDR: i64 = 0x20;
const REG_LIS2DW_CTRL_REG2_ADDR: i64 = 0x21;
const REG_LIS2DW_CTRL_REG4_ADDR: i64 = 0x23;
const REG_LIS2DW_CTRL_REG5_ADDR: i64 = 0x24;
const REG_LIS2DW_CTRL_REG6_ADDR: i64 = 0x25;
const REG_LIS2DW_FIFO_CTRL: i64 = 0x2E;
const REG_MOD_READ: i64 = 0x80;

const LIS2DW_DEV_ID: i64 = 0x44;
const LIS3DH_DEV_ID: i64 = 0x33;

const LIS_I2C_ADDR: i64 = 0x19;

// Right shift for left justified registers.
const FREEFALL_ACCEL: f64 = 9.80665;
const LIS2DW_SCALE: f64 = FREEFALL_ACCEL * 1.952 / 4.0;
const LIS3DH_SCALE: f64 = FREEFALL_ACCEL * 11.718 / 16.0;

pub const LIS2DW_TYPE: &str = "LIS2DW";
pub const LIS3DH_TYPE: &str = "LIS3DH";

/// the spi or i2c bus of the chip
enum Bus {
    Spi(Rc<McuSpi>),
    I2c(Rc<McuI2c>),
}

impl Bus {
    fn type_name(&self) -> &'static str {
        match self {
            Bus::Spi(_) => "spi",
            Bus::I2c(_) => "i2c",
        }
    }
    fn get_mcu(&self) -> Rc<Mcu> {
        match self {
            Bus::Spi(b) => b.get_mcu(),
            Bus::I2c(b) => b.get_mcu(),
        }
    }
    fn get_oid(&self) -> i64 {
        match self {
            Bus::Spi(b) => b.get_oid(),
            Bus::I2c(b) => b.get_oid(),
        }
    }
    fn get_command_queue(&self) -> crate::chelper::serialqueue::CommandQueue {
        match self {
            Bus::Spi(b) => b.get_command_queue(),
            Bus::I2c(b) => b.get_command_queue(),
        }
    }
}

// Printer class that controls LIS2DW chip
pub struct LIS2DW {
    printer: Rc<Printer>,
    cmd_helper: RefCell<Option<Rc<AccelCommandHelper>>>,
    pub(crate) lis_type: &'static str,
    axes_map: AxisMap,
    data_rate: i64,
    bus: Bus,
    mcu: Rc<Mcu>,
    oid: i64,
    query_lis2dw_cmd: RefCell<Option<CommandWrapper>>,
    ffreader: FixedFreqReader,
    last_error_count: Cell<i64>,
    pub(crate) batch_bulk: RefCell<Option<Rc<BatchBulkHelper>>>,
    pub name: String,
}

impl LIS2DW {
    pub fn new(config: &ConfigWrapper, lis_type: &'static str) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // AccelCommandHelper(config, self): the chip is set once built
        let cmd_helper = AccelCommandHelper::new(config)?;
        let (axes_map, data_rate) = if lis_type == LIS2DW_TYPE {
            (read_axes_map(config, LIS2DW_SCALE, LIS2DW_SCALE, LIS2DW_SCALE)?, 1600)
        } else {
            (read_axes_map(config, LIS3DH_SCALE, LIS3DH_SCALE, LIS3DH_SCALE)?, 1344)
        };
        // Check for spi or i2c
        let bus = if config.get_opt("cs_pin")?.is_some() {
            Bus::Spi(mcu_spi_from_config(config, 3, "cs_pin", 5000000, None, false)?)
        } else {
            Bus::I2c(mcu_i2c_from_config(config, Some(LIS_I2C_ADDR), 400000)?)
        };
        // Setup mcu sensor_lis2dw bulk query code
        let mcu = bus.get_mcu();
        let oid = mcu.create_oid();
        mcu.add_config_cmd(
            &format!("config_lis2dw oid={} bus_oid={} bus_oid_type={} lis_chip_type={}", oid, bus.get_oid(), bus.type_name(), lis_type),
            false,
            false,
        );
        mcu.add_config_cmd(&format!("query_lis2dw oid={} rest_ticks=0", oid), false, true);
        let slot: Rc<RefCell<Weak<LIS2DW>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        mcu.register_config_callback(Rc::new(move || match sl.borrow().upgrade() {
            Some(l) => l.build_config(),
            None => Ok(()),
        }));
        // Bulk sample message reading
        let chip_smooth = data_rate as f64 * BATCH_UPDATES * 2.0;
        let ffreader = FixedFreqReader::new(&mcu, chip_smooth, "<hhh");
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let l = Rc::new(LIS2DW {
            printer: printer.clone(),
            cmd_helper: RefCell::new(None),
            lis_type,
            axes_map,
            data_rate,
            bus,
            mcu,
            oid,
            query_lis2dw_cmd: RefCell::new(None),
            ffreader,
            last_error_count: Cell::new(0),
            batch_bulk: RefCell::new(None),
            name: name.clone(),
        });
        *slot.borrow_mut() = Rc::downgrade(&l);
        let chip: Rc<dyn AccelChip> = l.clone();
        cmd_helper.set_chip(Rc::downgrade(&chip));
        *l.cmd_helper.borrow_mut() = Some(cmd_helper);
        // Process messages in batches
        let (w1, w2, w3) = (Rc::downgrade(&l), Rc::downgrade(&l), Rc::downgrade(&l));
        let batch_bulk = BatchBulkHelper::new(
            &printer,
            Rc::new(move |eventtime| match w1.upgrade() {
                Some(l) => l.process_batch(eventtime),
                None => Ok(PyVal::dict()),
            }),
            Some(Rc::new(move || match w2.upgrade() {
                Some(l) => l.start_measurements(),
                None => Ok(()),
            })),
            Some(Rc::new(move || match w3.upgrade() {
                Some(l) => l.finish_measurements(),
                None => Ok(()),
            })),
            BATCH_UPDATES,
        );
        batch_bulk.add_mux_endpoint("lis2dw/dump_lis2dw", "sensor", &name, accel_header())?;
        *l.batch_bulk.borrow_mut() = Some(batch_bulk);
        Ok(l)
    }

    fn build_config(&self) -> KResult<()> {
        let cmdqueue = self.bus.get_command_queue();
        *self.query_lis2dw_cmd.borrow_mut() = Some(self.mcu.lookup_command("query_lis2dw oid=%c rest_ticks=%u", Some(cmdqueue.clone()))?);
        self.ffreader.setup_query_command("query_lis2dw_status oid=%c", self.oid, Some(cmdqueue))
    }

    fn check_connected(&self) -> KResult<()> {
        if self.mcu.non_critical_disconnected.get() {
            return Err(KError::command(format!(
                "LIS2DW: {} could not connect because mcu: {} is non_critical_disconnected!",
                self.name,
                self.mcu.get_name()
            )));
        }
        Ok(())
    }

    fn query_cmd(&self) -> KResult<std::cell::Ref<'_, CommandWrapper>> {
        let c = self.query_lis2dw_cmd.borrow();
        if c.is_none() {
            return Err(KError::Other("'NoneType' object has no attribute 'send'".into()));
        }
        Ok(std::cell::Ref::map(c, |c| c.as_ref().unwrap()))
    }

    // Start, stop, and process message batches
    fn start_measurements(&self) -> KResult<()> {
        // In case of miswiring, testing LIS2DW device ID prevents treating
        // noise or wrong signal as a correctly initialized device
        let dev_id = self.read_reg(REG_LIS2DW_WHO_AM_I_ADDR)?;
        logging::info(&format!("{}_dev_id: {:x}", self.lis_type, dev_id));
        if self.lis_type == LIS2DW_TYPE {
            if dev_id != LIS2DW_DEV_ID {
                return Err(KError::command(format!(
                    "Invalid lis2dw id (got {:x} vs {:x}).\nThis is generally indicative of connection problems\n(e.g. faulty wiring) or a faulty lis2dw chip.",
                    dev_id, LIS2DW_DEV_ID
                )));
            }
            if matches!(self.bus, Bus::Spi(_)) {
                // Disable I2C
                self.set_reg(REG_LIS2DW_CTRL_REG2_ADDR, 0x06)?;
            }
            // Setup chip in requested query rate
            // ODR/2, +-16g, low-pass filter, Low-noise abled
            self.set_reg(REG_LIS2DW_CTRL_REG6_ADDR, 0x34)?;
            // Continuous mode: If the FIFO is full
            // the new sample overwrites the older sample.
            self.set_reg(REG_LIS2DW_FIFO_CTRL, 0xC0)?;
            // High-Performance / Low-Power mode 1600/200 Hz
            // High-Performance Mode (14-bit resolution)
            self.set_reg(REG_LIS2DW_CTRL_REG1_ADDR, 0x94)?;
        } else {
            if dev_id != LIS3DH_DEV_ID {
                return Err(KError::command(format!(
                    "Invalid lis3dh id (got {:x} vs {:x}).\nThis is generally indicative of connection problems\n(e.g. faulty wiring) or a faulty lis3dh chip.",
                    dev_id, LIS3DH_DEV_ID
                )));
            }
            // High Resolution / Low Power mode 1344/5376 Hz
            // High Resolution mode (12-bit resolution)
            // Enable X Y Z axes
            self.set_reg(REG_LIS2DW_CTRL_REG1_ADDR, 0x97)?;
            // Disable all filtering
            self.set_reg(REG_LIS2DW_CTRL_REG2_ADDR, 0)?;
            // Set +-16g, High Resolution mode
            self.set_reg(REG_LIS2DW_CTRL_REG4_ADDR, 0x38)?;
            // Enable FIFO
            self.set_reg(REG_LIS2DW_CTRL_REG5_ADDR, 0x40)?;
            // Stream mode
            self.set_reg(REG_LIS2DW_FIFO_CTRL, 0x80)?;
        }
        // Start bulk reading
        let rest_ticks = self.mcu.seconds_to_clock(4.0 / self.data_rate as f64);
        self.query_cmd()?.send(&[Val::Int(self.oid), Val::Int(rest_ticks)], 0, 0)?;
        if self.lis_type == LIS2DW_TYPE {
            self.set_reg(REG_LIS2DW_FIFO_CTRL, 0xC0)?;
        } else {
            self.set_reg(REG_LIS2DW_FIFO_CTRL, 0x80)?;
        }
        logging::info(&format!("{} starting '{}' measurements", self.lis_type, self.name));
        // Initialize clock tracking
        self.ffreader.note_start()?;
        self.last_error_count.set(0);
        Ok(())
    }

    fn finish_measurements(&self) -> KResult<()> {
        // Halt bulk reading
        self.set_reg(REG_LIS2DW_FIFO_CTRL, 0x00)?;
        self.query_cmd()?.send_wait_ack(&[Val::Int(self.oid), Val::Int(0)], 0, 0)?;
        self.ffreader.note_end()?;
        logging::info(&format!("{} finished '{}' measurements", self.lis_type, self.name));
        self.set_reg(REG_LIS2DW_FIFO_CTRL, 0x00)
    }

    fn process_batch(&self, _eventtime: f64) -> KResult<PyVal> {
        let samples = self.ffreader.pull_samples()?;
        // Measurement decoding
        let samples: Vec<[f64; 4]> = samples.iter().map(|(ptime, f)| convert_sample(&self.axes_map, *ptime, [f[0], f[1], f[2]])).collect();
        Ok(batch_msg(&samples, self.last_error_count.get(), self.ffreader.get_last_overflows()))
    }
}

impl AccelChip for LIS2DW {
    fn start_internal_client(&self) -> KResult<Rc<AccelQueryHelper>> {
        self.check_connected()?;
        let aqh = AccelQueryHelper::new(&self.printer)?;
        let a = aqh.clone();
        let bb = self.batch_bulk.borrow().clone().ok_or_else(|| KError::Other("lis2dw batch helper missing".into()))?;
        bb.add_client(Rc::new(move |msg| a.handle_batch(msg)))?;
        Ok(aqh)
    }

    fn read_reg(&self, reg: i64) -> KResult<i64> {
        let (params, idx) = match &self.bus {
            Bus::Spi(spi) => (spi.spi_transfer(&[(reg | REG_MOD_READ) as u8, 0x00], 0, 0)?, 1),
            Bus::I2c(i2c) => (i2c.i2c_read(&[reg as u8], 1, true)?, 0),
        };
        match params.get("response") {
            Some(Val::Bytes(b)) if b.len() > idx => Ok(b[idx] as i64),
            _ => Err(KError::Other("bytearray index out of range".into())),
        }
    }

    fn set_reg(&self, reg: i64, val: i64) -> KResult<()> {
        let data = [reg as u8, (val & 0xFF) as u8];
        match &self.bus {
            Bus::Spi(spi) => spi.spi_send(&data, 0, 0)?,
            Bus::I2c(i2c) => i2c.i2c_write(&data, 0, 0)?,
        }
        let stored_val = self.read_reg(reg)?;
        if stored_val != val {
            return Err(KError::command(format!(
                "Failed to set LIS2DW register [0x{:x}] to 0x{:x}: got 0x{:x}. This is generally indicative of connection problems (e.g. faulty wiring) or a faulty lis2dw chip.",
                reg, val, stored_val
            )));
        }
        Ok(())
    }

    fn chip_name(&self) -> String {
        self.name.clone()
    }
}

impl PrinterObject for LIS2DW {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        if let Some(bb) = self.batch_bulk.borrow_mut().take() {
            bb.teardown();
        }
        *self.cmd_helper.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(LIS2DW::new(config, LIS2DW_TYPE)?)
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(LIS2DW::new(config, LIS2DW_TYPE)?)
}
