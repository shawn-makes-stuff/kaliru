// Code for handling printer nozzle extruders
// (port of klippy/kinematics/extruder.py)
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::chelper::itersolve::SkRef;
use crate::chelper::kin_extruder::{extruder_set_pressure_advance, extruder_stepper_alloc};
use crate::chelper::trapq::TrapQRef;
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::danger_options::get_danger_options;
use crate::extras::heaters::{load_heaters, Heater, PrinterHeaters};
use crate::extras::motion_queuing::PrinterMotionQueuing;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::stepper::{printer_stepper, McuStepper};
use crate::toolhead::{ExtraAxis, ExtraAxisRef, Move, ToolHead};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::f64::consts::PI;
use std::rc::{Rc, Weak};

const CMD_SET_PRESSURE_ADVANCE_HELP: &str = "Set pressure advance parameters";
const CMD_SET_E_ROTATION_DISTANCE_HELP: &str = "Set extruder rotation distance";
const CMD_SYNC_EXTRUDER_MOTION_HELP: &str = "Set extruder stepper motion queue";
const CMD_ACTIVATE_EXTRUDER_HELP: &str = "Change the active extruder";
const CMD_SET_NOZZLE_DIAMETER_HELP: &str = "Set nozzle diameter";

/// the PrinterExtruder behind the toolhead's active extra axis 0
fn as_printer_extruder(ea: &ExtraAxisRef) -> Option<&PrinterExtruder> {
    ea.as_any().downcast_ref::<PrinterExtruder>()
}

/// "'X' object has no attribute 'name'" for a non-PrinterExtruder axis
fn no_attribute(ea: &ExtraAxisRef, attr: &str) -> KError {
    let cls = if ea.as_any().is::<DummyExtruder>() { "DummyExtruder" } else { "object" };
    KError::Other(format!("'{}' object has no attribute '{}'", cls, attr))
}

pub struct ExtruderStepper {
    printer: Rc<Printer>,
    pub name: String,
    pressure_advance: Cell<f64>,
    pressure_advance_smooth_time: Cell<f64>,
    config_pa: f64,
    config_smooth_time: f64,
    per_move_pressure_advance: bool,
    pub stepper: Rc<McuStepper>,
    sk_extruder: SkRef,
    motion_queue: RefCell<Option<String>>,
}

impl ExtruderStepper {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        let config_pa = config.getfloat("pressure_advance", 0.0, Lim::new().min(0.0))?;
        let config_smooth_time = config.getfloat(
            "pressure_advance_smooth_time",
            0.040,
            Lim::new().above(0.0).max(get_danger_options().override_pressure_advance_smooth_time_max),
        )?;
        let per_move_pressure_advance = config.getboolean("per_move_pressure_advance", false)?;
        // Setup stepper
        let stepper = printer_stepper(config, false)?;
        let sk_extruder = extruder_stepper_alloc();
        stepper.set_stepper_kinematics(Some(sk_extruder.clone()));
        let es = Rc::new(ExtruderStepper {
            printer: printer.clone(),
            name: name.clone(),
            pressure_advance: Cell::new(0.0),
            pressure_advance_smooth_time: Cell::new(0.0),
            config_pa,
            config_smooth_time,
            per_move_pressure_advance,
            stepper,
            sk_extruder,
            motion_queue: RefCell::new(None),
        });
        // Register commands
        let e = es.clone();
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| e.set_pressure_advance(e.config_pa, e.config_smooth_time)),
        );
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        if name == "extruder" {
            let e = es.clone();
            gcode.register_mux_command(
                "SET_PRESSURE_ADVANCE",
                "EXTRUDER",
                None,
                Rc::new(move |g| e.cmd_default_set_pressure_advance(g)),
                Some(CMD_SET_PRESSURE_ADVANCE_HELP),
            )?;
        }
        let e = es.clone();
        gcode.register_mux_command(
            "SET_PRESSURE_ADVANCE",
            "EXTRUDER",
            Some(&name),
            Rc::new(move |g| e.cmd_set_pressure_advance(g)),
            Some(CMD_SET_PRESSURE_ADVANCE_HELP),
        )?;
        let e = es.clone();
        gcode.register_mux_command(
            "SET_EXTRUDER_ROTATION_DISTANCE",
            "EXTRUDER",
            Some(&name),
            Rc::new(move |g| e.cmd_set_e_rotation_distance(g)),
            Some(CMD_SET_E_ROTATION_DISTANCE_HELP),
        )?;
        let e = es.clone();
        gcode.register_mux_command(
            "SYNC_EXTRUDER_MOTION",
            "EXTRUDER",
            Some(&name),
            Rc::new(move |g| e.cmd_sync_extruder_motion(g)),
            Some(CMD_SYNC_EXTRUDER_MOTION_HELP),
        )?;
        Ok(es)
    }

    pub fn get_status(&self, _eventtime: f64) -> IndexMap<String, PyVal> {
        let mut d = IndexMap::new();
        d.insert("pressure_advance".to_string(), PyVal::Float(self.pressure_advance.get()));
        d.insert("smooth_time".to_string(), PyVal::Float(self.pressure_advance_smooth_time.get()));
        d.insert("motion_queue".to_string(), self.motion_queue.borrow().clone().map(PyVal::Str).unwrap_or(PyVal::None));
        d
    }

    pub fn find_past_position(&self, print_time: f64) -> f64 {
        let mcu_pos = self.stepper.get_past_mcu_position(print_time);
        self.stepper.mcu_to_commanded_position(mcu_pos)
    }

    pub fn sync_to_extruder(&self, extruder_name: &str) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        toolhead.flush_step_generation()?;
        if extruder_name.is_empty() {
            self.stepper.set_trapq(None);
            *self.motion_queue.borrow_mut() = None;
            return Ok(());
        }
        let extruder = self.printer.lookup_object_opt(extruder_name).and_then(downcast_rc::<PrinterExtruder>);
        let Some(extruder) = extruder else {
            return Err(KError::command(format!("'{}' is not a valid extruder.", extruder_name)));
        };
        self.stepper.set_position(&[extruder.last_position.get(), 0.0, 0.0]);
        self.stepper.set_trapq(Some(extruder.get_trapq_ref()));
        *self.motion_queue.borrow_mut() = Some(extruder_name.to_string());
        Ok(())
    }

    pub fn set_rotation_distance(&self, rotation_dist: f64) {
        self.stepper.set_rotation_distance(rotation_dist);
    }

    /// `_, rotation_dist = self.stepper.get_rotation_distance()`: python
    /// unpacks the second item (steps_per_rotation)
    pub fn get_rotation_distance(&self) -> f64 {
        self.stepper.get_rotation_distance().1
    }

    fn set_pressure_advance(&self, pressure_advance: f64, smooth_time: f64) -> KResult<()> {
        let mut old_smooth_time = self.pressure_advance_smooth_time.get();
        if self.pressure_advance.get() == 0.0 {
            old_smooth_time = 0.0;
        }
        let mut new_smooth_time = smooth_time;
        if pressure_advance == 0.0 {
            new_smooth_time = 0.0;
        }
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        if new_smooth_time != old_smooth_time {
            // Need full kinematic flush to change the smooth time
            toolhead.flush_step_generation()?;
            extruder_set_pressure_advance(&self.sk_extruder, 0.0, pressure_advance, new_smooth_time);
            let motion_queuing = self.printer.lookup::<PrinterMotionQueuing>("motion_queuing")?;
            motion_queuing.check_step_generation_scan_windows();
        } else {
            let sk = self.sk_extruder.clone();
            toolhead.register_lookahead_callback(Rc::new(move |print_time| {
                extruder_set_pressure_advance(&sk, print_time, pressure_advance, new_smooth_time);
                Ok(())
            }))?;
        }

        self.pressure_advance.set(pressure_advance);
        self.pressure_advance_smooth_time.set(smooth_time);
        Ok(())
    }

    fn cmd_default_set_pressure_advance(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let extruder = self.printer.lookup::<ToolHead>("toolhead")?.get_extruder();
        let pe = as_printer_extruder(&extruder).ok_or_else(|| no_attribute(&extruder, "extruder_stepper"))?;
        let Some(es) = pe.extruder_stepper.clone() else {
            return Err(gcmd.error("Active extruder does not have a stepper"));
        };
        let strapq = es.stepper.get_trapq();
        let same = strapq.as_ref().map(|t| Rc::ptr_eq(t, &pe.trapq)).unwrap_or(false);
        if !same {
            return Err(gcmd.error("Unable to infer active extruder stepper"));
        }
        es.cmd_set_pressure_advance(gcmd)
    }

    fn cmd_set_pressure_advance(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let pressure_advance = gcmd.get_float("ADVANCE", Some(self.pressure_advance.get()), Lim::new().min(0.0))?;
        let smooth_time = gcmd.get_float(
            "SMOOTH_TIME",
            Some(self.pressure_advance_smooth_time.get()),
            Lim::new().min(0.0).max(get_danger_options().override_pressure_advance_smooth_time_max),
        )?;
        self.set_pressure_advance(pressure_advance, smooth_time)?;

        if get_danger_options().log_pressure_advance_changes {
            let msg = [format!("pressure_advance: {:.6}", pressure_advance), format!("pressure_advance_smooth_time: {:.6}", smooth_time)];
            self.printer.set_rollover_info(&self.name, Some(&format!("{}: {}", self.name, msg.join(" "))), true);
            gcmd.respond_info(&msg.join("\n"), false);
        }
        Ok(())
    }

    fn cmd_set_e_rotation_distance(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mut rotation_dist = match gcmd.get_float_opt("DISTANCE", Lim::new())? {
            Some(mut rotation_dist) => {
                if rotation_dist == 0.0 {
                    return Err(gcmd.error("Rotation distance can not be zero"));
                }
                let (_invert_dir, orig_invert_dir) = self.stepper.get_dir_inverted();
                let mut next_invert_dir = orig_invert_dir;
                if rotation_dist < 0.0 {
                    next_invert_dir = !orig_invert_dir;
                    rotation_dist = -rotation_dist;
                }
                let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
                toolhead.flush_step_generation()?;
                self.stepper.set_rotation_distance(rotation_dist);
                self.stepper.set_dir_inverted(next_invert_dir)?;
                rotation_dist
            }
            None => self.stepper.get_rotation_distance().0,
        };
        let (invert_dir, orig_invert_dir) = self.stepper.get_dir_inverted();
        if invert_dir != orig_invert_dir {
            rotation_dist = -rotation_dist;
        }
        gcmd.respond_info(&format!("Extruder '{}' rotation distance set to {:.6}", self.name, rotation_dist), true);
        Ok(())
    }

    fn cmd_sync_extruder_motion(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let ename = gcmd.get("MOTION_QUEUE")?;
        self.sync_to_extruder(&ename)?;
        gcmd.respond_info(&format!("Extruder '{}' now syncing with '{}'", self.name, ename), true);
        Ok(())
    }
}

// Tracking for hotend heater, extrusion motion queue, and extruder stepper
pub struct PrinterExtruder {
    me: Weak<PrinterExtruder>,
    printer: Rc<Printer>,
    pub(crate) name: String,
    pub last_position: Cell<f64>,
    pub(crate) heater: Rc<Heater>,
    pub(crate) nozzle_diameter: Cell<f64>,
    pub(crate) filament_area: f64,
    pub(crate) config_max_cross_section: Option<f64>,
    pub(crate) max_extrude_ratio: Cell<f64>,
    pub max_e_velocity: f64,
    pub max_e_accel: f64,
    pub max_e_dist: f64,
    pub(crate) instant_corner_v: f64,
    trapq: TrapQRef,
    pub extruder_stepper: Option<Rc<ExtruderStepper>>,
}

impl PrinterExtruder {
    pub fn new(config: &ConfigWrapper, extruder_num: usize) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().to_string();
        // Setup hotend heater
        let pheaters = load_heaters(config)?;
        let gcode_id = format!("T{}", extruder_num);
        let heater = pheaters.setup_heater(config, Some(&gcode_id))?;
        // Setup kinematic checks
        let nozzle_diameter = config.getfloat("nozzle_diameter", Dflt::Required, Lim::new().above(0.0))?;
        let filament_diameter = config.getfloat("filament_diameter", Dflt::Required, Lim::new().min(nozzle_diameter))?;
        let filament_area = PI * (filament_diameter * 0.5).powf(2.0);
        let def_max_cross_section = 4.0 * nozzle_diameter.powf(2.0);
        let def_max_extrude_ratio = def_max_cross_section / filament_area;
        // None unless set in the config, so SET_NOZZLE_DIAMETER knows
        // whether to recalculate the default
        let config_max_cross_section = config.getfloat_opt("max_extrude_cross_section", Lim::new().above(0.0))?;
        let max_cross_section = config_max_cross_section.unwrap_or(def_max_cross_section);
        let max_extrude_ratio = max_cross_section / filament_area;
        logging::info(&format!("Extruder max_extrude_ratio={:.6}", max_extrude_ratio));
        let toolhead = printer.lookup::<ToolHead>("toolhead")?;
        let (max_velocity, max_accel) = toolhead.get_max_velocity();
        let max_e_velocity =
            config.getfloat("max_extrude_only_velocity", max_velocity * def_max_extrude_ratio, Lim::new().above(0.0))?;
        let max_e_accel = config.getfloat("max_extrude_only_accel", max_accel * def_max_extrude_ratio, Lim::new().above(0.0))?;
        let max_e_dist = config.getfloat("max_extrude_only_distance", 50.0, Lim::new().min(0.0))?;
        let instant_corner_v = config.getfloat("instantaneous_corner_velocity", 1.0, Lim::new().min(0.0))?;
        // Setup extruder trapq (trapezoidal motion queue)
        let motion_queuing = printer
            .load_object(config, "motion_queuing", true)?
            .and_then(downcast_rc::<PrinterMotionQueuing>)
            .ok_or_else(|| KError::Other("motion_queuing has an unexpected type".into()))?;
        let trapq = motion_queuing.allocate_trapq();

        // Setup extruder stepper
        let mut extruder_stepper = None;
        if config.get_opt("step_pin")?.is_some() || config.get_opt("dir_pin")?.is_some() || config.get_opt("rotation_distance")?.is_some() {
            let es = ExtruderStepper::new(config)?;
            es.stepper.set_trapq(Some(trapq.clone()));
            extruder_stepper = Some(es);
        }
        let pe = Rc::new_cyclic(|me| PrinterExtruder {
            me: me.clone(),
            printer: printer.clone(),
            name: name.clone(),
            last_position: Cell::new(0.0),
            heater,
            nozzle_diameter: Cell::new(nozzle_diameter),
            filament_area,
            config_max_cross_section,
            max_extrude_ratio: Cell::new(max_extrude_ratio),
            max_e_velocity,
            max_e_accel,
            max_e_dist,
            instant_corner_v,
            trapq,
            extruder_stepper,
        });
        // Register commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        if name == "extruder" {
            toolhead.set_extruder(pe.clone(), 0.0);
            let p = pe.clone();
            gcode.register_command("M104", Some(Rc::new(move |g| p.cmd_m104(g, false))), false, None)?;
            let p = pe.clone();
            gcode.register_command("M109", Some(Rc::new(move |g| p.cmd_m104(g, true))), false, None)?;
            let p = pe.clone();
            gcode.register_command("M302", Some(Rc::new(move |g| p.cmd_m302(g))), false, None)?;
            let p = pe.clone();
            gcode.register_mux_command(
                "SET_NOZZLE_DIAMETER",
                "EXTRUDER",
                None,
                Rc::new(move |g| p.cmd_default_set_nozzle_diameter(g)),
                Some(CMD_SET_NOZZLE_DIAMETER_HELP),
            )?;
        }
        let p = pe.clone();
        gcode.register_mux_command(
            "ACTIVATE_EXTRUDER",
            "EXTRUDER",
            Some(&name),
            Rc::new(move |g| p.cmd_activate_extruder(g)),
            Some(CMD_ACTIVATE_EXTRUDER_HELP),
        )?;
        let p = pe.clone();
        gcode.register_mux_command(
            "SET_NOZZLE_DIAMETER",
            "EXTRUDER",
            Some(&name),
            Rc::new(move |g| p.cmd_set_nozzle_diameter(g)),
            Some(CMD_SET_NOZZLE_DIAMETER_HELP),
        )?;
        Ok(pe)
    }

    fn rc(&self) -> Rc<PrinterExtruder> {
        self.me.upgrade().expect("extruder")
    }

    pub fn get_trapq_ref(&self) -> TrapQRef {
        self.trapq.clone()
    }

    pub fn get_status(&self, eventtime: f64) -> PyVal {
        let mut sts = match self.heater.get_status(eventtime) {
            PyVal::Dict(d) => d,
            _ => IndexMap::new(),
        };
        sts.insert("can_extrude".to_string(), PyVal::Bool(self.heater.can_extrude.get()));
        sts.insert("nozzle_diameter".to_string(), PyVal::Float(self.nozzle_diameter.get()));
        if let Some(es) = &self.extruder_stepper {
            sts.extend(es.get_status(eventtime));
        }
        PyVal::Dict(sts)
    }

    /// lookup_object(section, None) of an extruder index (M104 T / M302 T)
    fn lookup_index(&self, index: i64) -> Option<PObj> {
        let section = if index != 0 { format!("extruder{}", index) } else { "extruder".to_string() };
        self.printer.lookup_object_opt(&section)
    }

    /// extruder.get_heater() of a looked up object
    fn obj_heater(obj: &PObj) -> KResult<Rc<Heater>> {
        match obj.as_any().downcast_ref::<PrinterExtruder>() {
            Some(pe) => Ok(pe.heater.clone()),
            None => Err(KError::Other("object has no attribute 'get_heater'".into())),
        }
    }

    fn cmd_m104(&self, gcmd: &GCodeCommand, wait: bool) -> KResult<()> {
        // Set Extruder Temperature
        let temp = gcmd.get_float("S", Some(0.0), Lim::new())?;
        let index = gcmd.get_int_opt("T", Lim::new().min(0))?;
        let heater = match index {
            Some(index) => match self.lookup_index(index) {
                None => {
                    if temp <= 0.0 {
                        return Ok(());
                    }
                    return Err(gcmd.error("Extruder not configured"));
                }
                Some(extruder) => Self::obj_heater(&extruder)?,
            },
            None => self.printer.lookup::<ToolHead>("toolhead")?.get_extruder().get_heater()?,
        };
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        pheaters.set_temperature(&heater, temp, wait)
    }

    fn cmd_m302(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let index = gcmd.get_int_opt("T", Lim::new().min(0))?;
        let heater = match index {
            Some(index) => match self.lookup_index(index) {
                // python passes the format arguments to gcmd.error() itself:
                // str() of the exception is the repr of its args tuple
                None => return Err(gcmd.error(format!("('Extruder%d not configured', ({},))", index))),
                Some(extruder) => Self::obj_heater(&extruder)?,
            },
            None => self.printer.lookup::<ToolHead>("toolhead")?.get_extruder().get_heater()?,
        };
        let cold_extrude = gcmd.get_int_opt("P", Lim::new().min(0).max(1))?;
        let min_extrude_temp = gcmd.get_float_opt("S", Lim::new().min(heater.min_temp).max(heater.max_temp))?;
        heater.set_cold_extrude(cold_extrude, min_extrude_temp)
    }

    fn cmd_activate_extruder(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let active = toolhead.get_extruder();
        let is_self = as_printer_extruder(&active).map(|p| std::ptr::eq(p, self)).unwrap_or(false);
        if is_self {
            gcmd.respond_info(&format!("Extruder {} already active", self.name), true);
            return Ok(());
        }
        gcmd.respond_info(&format!("Activating extruder {}", self.name), true);
        toolhead.flush_step_generation()?;
        toolhead.set_extruder(self.rc(), self.last_position.get());
        self.printer.send_event("extruder:activate_extruder", &[])
    }

    fn cmd_default_set_nozzle_diameter(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let extruder = self.printer.lookup::<ToolHead>("toolhead")?.get_extruder();
        let pe = as_printer_extruder(&extruder).ok_or_else(|| no_attribute(&extruder, "cmd_SET_NOZZLE_DIAMETER"))?;
        pe.cmd_set_nozzle_diameter(gcmd)
    }

    fn cmd_set_nozzle_diameter(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let diameter = gcmd.get_float("DIAMETER", None, Lim::new().above(0.0))?;
        let cross_section = gcmd.get_float_opt("MAX_EXTRUDE_CROSS_SECTION", Lim::new().above(0.0))?;
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        toolhead.flush_step_generation()?;
        self.nozzle_diameter.set(diameter);
        let cross_section = cross_section.or(self.config_max_cross_section).unwrap_or_else(|| 4.0 * diameter.powf(2.0));
        self.max_extrude_ratio.set(cross_section / self.filament_area);
        gcmd.respond_info(
            &format!("Extruder '{}': nozzle_diameter={:.3} max_extrude_cross_section={:.3}", self.name, diameter, cross_section),
            true,
        );
        Ok(())
    }
}

impl ExtraAxis for PrinterExtruder {
    fn check_move(&self, mv: &mut Move, ea_index: usize) -> KResult<()> {
        if !self.heater.can_extrude.get() {
            return Err(KError::command("Extrude below minimum temp\nSee the 'min_extrude_temp' config option for details"));
        }
        let axis_r = mv.axes_r[ea_index];
        let axis_d = mv.axes_d[ea_index];
        if (mv.axes_d[0] == 0.0 && mv.axes_d[1] == 0.0) || axis_r < 0.0 {
            // Extrude only move (or retraction move) - limit accel and velocity
            if axis_d.abs() > self.max_e_dist {
                return Err(KError::command(format!(
                    "Extrude only move too long ({:.3}mm vs {:.3}mm)\nSee the 'max_extrude_only_distance' config option for details",
                    axis_d, self.max_e_dist
                )));
            }
            let inv_extrude_r = 1.0 / axis_r.abs();
            mv.limit_speed(self.max_e_velocity * inv_extrude_r, self.max_e_accel * inv_extrude_r);
        } else if axis_r > self.max_extrude_ratio.get() {
            if axis_d <= self.nozzle_diameter.get() * self.max_extrude_ratio.get() {
                // Permit extrusion if amount extruded is tiny
                return Ok(());
            }
            let area = axis_r * self.filament_area;
            logging::debug(&format!(
                "Overextrude: {} vs {} (area={:.3} dist={:.3})",
                crate::pyfmt::py_float_repr(axis_r),
                crate::pyfmt::py_float_repr(self.max_extrude_ratio.get()),
                area,
                mv.move_d
            ));
            return Err(KError::command(format!(
                "Move exceeds maximum extrusion ({:.3}mm^2 vs {:.3}mm^2)\nSee the 'max_extrude_cross_section' config option for details",
                area,
                self.max_extrude_ratio.get() * self.filament_area
            )));
        }
        Ok(())
    }

    fn find_past_position(&self, print_time: f64) -> f64 {
        match &self.extruder_stepper {
            None => 0.0,
            Some(es) => es.find_past_position(print_time),
        }
    }

    fn calc_junction(&self, prev_move: &Move, mv: &Move, ea_index: usize) -> f64 {
        let diff_r = mv.axes_r[ea_index] - prev_move.axes_r[ea_index];
        if diff_r != 0.0 {
            return (self.instant_corner_v / diff_r.abs()).powf(2.0);
        }
        mv.max_cruise_v2
    }

    fn process_move(&self, print_time: f64, mv: &Move, ea_index: usize) -> KResult<()> {
        let axis_r = mv.axes_r[ea_index];
        let accel = mv.accel * axis_r;
        let start_v = mv.start_v * axis_r;
        let cruise_v = mv.cruise_v * axis_r;
        let mut pressure_advance = 0.0;
        let mut use_pa_from_trapq = 0.0;
        if let Some(es) = &self.extruder_stepper {
            if es.per_move_pressure_advance {
                use_pa_from_trapq = 1.0;
            }
            if axis_r > 0.0 && (mv.axes_d[0] != 0.0 || mv.axes_d[1] != 0.0) {
                pressure_advance = es.pressure_advance.get();
            }
        }
        // Queue movement (x is extruder movement, y is pressure advance flag)
        self.trapq.borrow_mut().append(
            print_time,
            mv.accel_t,
            mv.cruise_t,
            mv.decel_t,
            mv.start_pos[ea_index],
            0.0,
            0.0,
            1.0,
            pressure_advance,
            use_pa_from_trapq,
            start_v,
            cruise_v,
            accel,
        );
        self.last_position.set(mv.end_pos[ea_index]);
        Ok(())
    }

    fn get_name(&self) -> String {
        self.name.clone()
    }

    fn get_trapq(&self) -> Option<TrapQRef> {
        Some(self.trapq.clone())
    }

    fn get_axis_gcode_id(&self) -> String {
        "E".into()
    }

    fn get_heater(&self) -> KResult<Rc<Heater>> {
        Ok(self.heater.clone())
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl PrinterObject for PrinterExtruder {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(PrinterExtruder::get_status(self, eventtime))
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        Some(self.heater.stats(eventtime))
    }
}

/// extruder.get_trapq() of an "extruderN" printer object
pub fn extruder_trapq(obj: &PObj) -> KResult<TrapQRef> {
    match obj.as_any().downcast_ref::<PrinterExtruder>() {
        Some(pe) => Ok(pe.trapq.clone()),
        None => Err(KError::Other("object has no attribute 'get_trapq'".into())),
    }
}

// Dummy extruder class used when a printer has no extruder at all
pub struct DummyExtruder {
    _printer: Rc<Printer>,
}

impl DummyExtruder {
    pub fn new(printer: &Rc<Printer>) -> Self {
        DummyExtruder { _printer: printer.clone() }
    }
}

impl ExtraAxis for DummyExtruder {
    fn check_move(&self, mv: &mut Move, _ea_index: usize) -> KResult<()> {
        Err(mv.move_error("Extrude when no extruder present"))
    }

    fn find_past_position(&self, _print_time: f64) -> f64 {
        0.0
    }

    fn calc_junction(&self, _prev_move: &Move, mv: &Move, _ea_index: usize) -> f64 {
        mv.max_cruise_v2
    }

    fn process_move(&self, _print_time: f64, _mv: &Move, _ea_index: usize) -> KResult<()> {
        // python's DummyExtruder has no process_move (AttributeError)
        Err(KError::Other("'DummyExtruder' object has no attribute 'process_move'".into()))
    }

    fn get_name(&self) -> String {
        String::new()
    }

    fn get_trapq(&self) -> Option<TrapQRef> {
        None
    }

    fn get_axis_gcode_id(&self) -> String {
        "E".into()
    }

    fn get_heater(&self) -> KResult<Rc<crate::extras::heaters::Heater>> {
        Err(KError::command("Extruder not configured"))
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn add_printer_objects(config: &ConfigWrapper) -> KResult<()> {
    let printer = config.get_printer();
    for i in 0..99 {
        let section = if i == 0 { "extruder".to_string() } else { format!("extruder{}", i) };
        if !config.has_section(&section) {
            break;
        }
        let pe = PrinterExtruder::new(&config.getsection(&section), i)?;
        printer.add_object(&section, pe)?;
    }
    Ok(())
}
