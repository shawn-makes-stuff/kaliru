// G-Code G1 movement commands (and associated coordinate manipulation)
// (port of klippy/extras/gcode_move.py)
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::homing::Homing;
use crate::gcode::{coord_pyval, GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_bool, py_float, py_float_repr};
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

/// a g-code move transform (toolhead, bed_mesh, skew_correction, ...)
pub trait MoveTransform {
    fn move_to(&self, newpos: &[f64], speed: f64) -> KResult<()>;
    fn get_position(&self) -> Vec<f64>;
}

pub type MoveTransformRef = Rc<dyn MoveTransform>;

impl MoveTransform for ToolHead {
    fn move_to(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        ToolHead::move_to(self, newpos, speed)
    }
    fn get_position(&self) -> Vec<f64> {
        ToolHead::get_position(self)
    }
}

#[derive(Clone)]
struct SavedState {
    absolute_coord: bool,
    absolute_extrude: bool,
    base_position: Vec<f64>,
    last_position: Vec<f64>,
    homing_position: Vec<f64>,
    speed: f64,
    speed_factor: f64,
    extrude_factor: f64,
}

pub struct GCodeMove {
    printer: Rc<Printer>,
    is_printer_ready: Cell<bool>,
    // G-Code coordinate manipulation
    pub(crate) absolute_coord: Cell<bool>,
    pub(crate) absolute_extrude: Cell<bool>,
    pub(crate) base_position: RefCell<Vec<f64>>,
    pub(crate) last_position: RefCell<Vec<f64>>,
    pub(crate) homing_position: RefCell<Vec<f64>>,
    axis_map: RefCell<IndexMap<String, usize>>,
    pub(crate) speed: Cell<f64>,
    pub(crate) speed_factor: Cell<f64>,
    pub(crate) extrude_factor: Cell<f64>,
    // G-Code state
    saved_states: RefCell<IndexMap<String, SavedState>>,
    move_transform: RefCell<Option<MoveTransformRef>>,
    /// move_with_transform / position_with_transform (None: no transform
    /// yet - position_with_transform returns [0.0, 0.0, 0.0, 0.0])
    with_transform: RefCell<Option<MoveTransformRef>>,
}

const CMD_SET_GCODE_OFFSET_HELP: &str = "Set a virtual offset to g-code positions";
const CMD_SAVE_GCODE_STATE_HELP: &str = "Save G-Code coordinate state";
const CMD_RESTORE_GCODE_STATE_HELP: &str = "Restore a previously saved G-Code state";
const CMD_GET_POSITION_HELP: &str = "Return information on the current location of the toolhead";

/// python's repr() of a list of floats
fn list_repr(v: &[f64]) -> String {
    format!("[{}]", v.iter().map(|f| py_float_repr(*f)).collect::<Vec<_>>().join(", "))
}

fn default_axis_map() -> IndexMap<String, usize> {
    [("X", 0), ("Y", 1), ("Z", 2), ("E", 3)].into_iter().map(|(k, v)| (k.to_string(), v)).collect()
}

type CmdFn = fn(&GCodeMove, &GCodeCommand) -> KResult<()>;

impl GCodeMove {
    /// get_status()["position"].e
    pub fn status_position_e(&self) -> f64 {
        self.last_position.borrow()[3]
    }

    /// get_status()["extrude_factor"]
    pub fn status_extrude_factor(&self) -> f64 {
        self.extrude_factor.get()
    }

    /// get_status()["homing_origin"] as a list
    pub fn homing_position(&self) -> Vec<f64> {
        self.homing_position.borrow().clone()
    }

    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gm = Rc::new(GCodeMove {
            printer: printer.clone(),
            is_printer_ready: Cell::new(false),
            absolute_coord: Cell::new(true),
            absolute_extrude: Cell::new(true),
            base_position: RefCell::new(vec![0.0; 4]),
            last_position: RefCell::new(vec![0.0; 4]),
            homing_position: RefCell::new(vec![0.0; 4]),
            axis_map: RefCell::new(default_axis_map()),
            speed: Cell::new(25.0),
            speed_factor: Cell::new(1.0 / 60.0),
            extrude_factor: Cell::new(1.0),
            saved_states: RefCell::new(IndexMap::new()),
            move_transform: RefCell::new(None),
            with_transform: RefCell::new(None),
        });
        // Register g-code commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let handlers: [(&str, CmdFn, Option<&str>); 13] = [
            ("G1", GCodeMove::cmd_g1, None),
            ("G20", GCodeMove::cmd_g20, None),
            ("G21", GCodeMove::cmd_g21, None),
            ("M82", GCodeMove::cmd_m82, None),
            ("M83", GCodeMove::cmd_m83, None),
            ("G90", GCodeMove::cmd_g90, None),
            ("G91", GCodeMove::cmd_g91, None),
            ("G92", GCodeMove::cmd_g92, None),
            ("M220", GCodeMove::cmd_m220, None),
            ("M221", GCodeMove::cmd_m221, None),
            ("SET_GCODE_OFFSET", GCodeMove::cmd_set_gcode_offset, Some(CMD_SET_GCODE_OFFSET_HELP)),
            ("SAVE_GCODE_STATE", GCodeMove::cmd_save_gcode_state, Some(CMD_SAVE_GCODE_STATE_HELP)),
            ("RESTORE_GCODE_STATE", GCodeMove::cmd_restore_gcode_state, Some(CMD_RESTORE_GCODE_STATE_HELP)),
        ];
        let reg = |cmd: &str, f: CmdFn, when_not_ready: bool, desc: Option<&str>| {
            let g = gm.clone();
            gcode.register_command(cmd, Some(Rc::new(move |gcmd| f(&g, gcmd))), when_not_ready, desc).map(|_| ())
        };
        for (cmd, f, desc) in handlers {
            reg(cmd, f, false, desc)?;
        }
        reg("G0", GCodeMove::cmd_g1, false, None)?;
        reg("M114", GCodeMove::cmd_m114, true, None)?;
        reg("GET_POSITION", GCodeMove::cmd_get_position, true, Some(CMD_GET_POSITION_HELP))?;
        // Register callbacks
        let on = |event: &str, f: fn(&GCodeMove) -> KResult<()>| {
            let g = gm.clone();
            printer.register_event_handler(event, Rc::new(move |_| f(&g)));
        };
        on("klippy:ready", GCodeMove::handle_ready);
        on("klippy:shutdown", |g| {
            g.is_printer_ready.set(false);
            Ok(())
        });
        on("klippy:analyze_shutdown", |g| {
            g.handle_analyze_shutdown();
            Ok(())
        });
        on("toolhead:set_position", GCodeMove::reset_last_position);
        on("toolhead:manual_move", GCodeMove::reset_last_position);
        on("toolhead:update_extra_axes", GCodeMove::update_extra_axes);
        on("gcode:command_error", GCodeMove::reset_last_position);
        on("extruder:activate_extruder", GCodeMove::handle_activate_extruder);
        let g = gm.clone();
        printer.register_event_handler(
            "homing:home_rails_end",
            Rc::new(move |args| {
                let homing_state = args.first().and_then(|a| a.downcast_ref::<Rc<Homing>>()).cloned();
                g.handle_home_rails_end(homing_state.as_ref())
            }),
        );
        Ok(gm)
    }

    fn handle_ready(&self) -> KResult<()> {
        self.is_printer_ready.set(true);
        if self.move_transform.borrow().is_none() {
            let toolhead: MoveTransformRef = self.printer.lookup::<ToolHead>("toolhead")?;
            *self.with_transform.borrow_mut() = Some(toolhead);
        }
        self.reset_last_position()
    }

    fn handle_analyze_shutdown(&self) {
        logging::info(&format!(
            "gcode state: absolute_coord={} absolute_extrude={} base_position={} last_position={} homing_position={} speed_factor={} extrude_factor={} speed={}",
            py_bool(self.absolute_coord.get()),
            py_bool(self.absolute_extrude.get()),
            list_repr(&self.base_position.borrow()),
            list_repr(&self.last_position.borrow()),
            list_repr(&self.homing_position.borrow()),
            py_float_repr(self.speed_factor.get()),
            py_float_repr(self.extrude_factor.get()),
            py_float_repr(self.speed.get()),
        ));
    }

    fn handle_activate_extruder(&self) -> KResult<()> {
        self.reset_last_position()?;
        self.extrude_factor.set(1.0);
        let e = self.last_position.borrow()[3];
        self.base_position.borrow_mut()[3] = e;
        Ok(())
    }

    fn handle_home_rails_end(&self, homing_state: Option<&Rc<Homing>>) -> KResult<()> {
        self.reset_last_position()?;
        if let Some(homing_state) = homing_state {
            for axis in homing_state.get_axes() {
                let hp = self.homing_position.borrow()[axis];
                self.base_position.borrow_mut()[axis] = hp;
            }
        }
        Ok(())
    }

    /// set_move_transform(transform, force); returns the old transform
    pub fn set_move_transform(&self, transform: MoveTransformRef, force: bool) -> KResult<Option<MoveTransformRef>> {
        if self.move_transform.borrow().is_some() && !force {
            return Err(KError::config("G-Code move transform already specified"));
        }
        let mut old_transform = self.move_transform.borrow().clone();
        if old_transform.is_none() {
            old_transform = self.printer.lookup_object_opt("toolhead").map(|_| {
                let th: MoveTransformRef = self.printer.lookup::<ToolHead>("toolhead").expect("toolhead");
                th
            });
        }
        *self.move_transform.borrow_mut() = Some(transform.clone());
        *self.with_transform.borrow_mut() = Some(transform);
        Ok(old_transform)
    }

    pub(crate) fn position_with_transform(&self) -> Vec<f64> {
        match self.with_transform.borrow().as_ref() {
            Some(t) => t.get_position(),
            None => vec![0.0; 4],
        }
    }

    pub(crate) fn move_with_transform(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        let t = self.with_transform.borrow().clone();
        match t {
            Some(t) => t.move_to(newpos, speed),
            None => Err(KError::Other("'NoneType' object is not callable".into())),
        }
    }

    fn get_gcode_position(&self) -> Vec<f64> {
        let lp = self.last_position.borrow();
        let bp = self.base_position.borrow();
        let mut p: Vec<f64> = lp.iter().zip(bp.iter()).map(|(l, b)| l - b).collect();
        p[3] /= self.extrude_factor.get();
        p.truncate(4);
        p
    }

    fn get_gcode_speed(&self) -> f64 {
        self.speed.get() / self.speed_factor.get()
    }

    fn get_gcode_speed_override(&self) -> f64 {
        self.speed_factor.get() * 60.0
    }

    /// saved_states[dst] = saved_states[src] (KeyError when src is missing)
    pub fn copy_saved_state(&self, dst: &str, src: &str) -> KResult<()> {
        let state = self.saved_states.borrow().get(src).cloned().ok_or_else(|| KError::Other(crate::pyfmt::py_str_repr(src)))?;
        self.saved_states.borrow_mut().insert(dst.to_string(), state);
        Ok(())
    }

    pub fn reset_last_position(&self) -> KResult<()> {
        if self.is_printer_ready.get() {
            let pos = self.position_with_transform();
            *self.last_position.borrow_mut() = pos;
        }
        Ok(())
    }

    fn update_extra_axes(&self) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let mut axis_map = default_axis_map();
        let extra_axes = toolhead.get_extra_axes();
        for (index, ea) in extra_axes.iter().enumerate() {
            let Some(ea) = ea else { continue };
            let gcode_id = ea.get_axis_gcode_id();
            // gcode_id in "FN" is a substring test ("" is in every string)
            if axis_map.contains_key(&gcode_id) || "FN".contains(gcode_id.as_str()) {
                continue;
            }
            axis_map.insert(gcode_id, index);
        }
        *self.axis_map.borrow_mut() = axis_map;
        {
            let mut bp = self.base_position.borrow_mut();
            bp.truncate(4);
            bp.extend(std::iter::repeat(0.0).take(extra_axes.len().saturating_sub(4)));
        }
        self.reset_last_position()
    }

    // G-Code movement commands
    pub fn cmd_g1(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Move
        let params = gcmd.get_command_parameters();
        let parse_error = || gcmd.error(format!("Unable to parse move '{}'", gcmd.get_commandline()));
        let axis_map = self.axis_map.borrow().clone();
        for (axis, &pos) in &axis_map {
            if let Some(value) = params.get(axis) {
                let mut v = py_float(value).ok_or_else(parse_error)?;
                let mut absolute_coord = self.absolute_coord.get();
                if axis == "E" {
                    v *= self.extrude_factor.get();
                    if !self.absolute_extrude.get() {
                        absolute_coord = false;
                    }
                }
                let mut lp = self.last_position.borrow_mut();
                if !absolute_coord {
                    // value relative to position of last move
                    lp[pos] += v;
                } else {
                    // value relative to base coordinate position
                    lp[pos] = v + self.base_position.borrow()[pos];
                }
            }
        }
        if let Some(f) = params.get("F") {
            let gcode_speed = py_float(f).ok_or_else(parse_error)?;
            if gcode_speed <= 0.0 {
                return Err(gcmd.error(format!("Invalid speed in '{}'", gcmd.get_commandline())));
            }
            self.speed.set(gcode_speed * self.speed_factor.get());
        }
        let lp = self.last_position.borrow().clone();
        self.move_with_transform(&lp, self.speed.get())
    }

    // G-Code coordinate manipulation
    fn cmd_g20(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Set units to inches
        Err(gcmd.error("Machine does not support G20 (inches) command"))
    }

    fn cmd_g21(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        // Set units to millimeters
        Ok(())
    }

    fn cmd_m82(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        // Use absolute distances for extrusion
        self.absolute_extrude.set(true);
        Ok(())
    }

    fn cmd_m83(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        // Use relative distances for extrusion
        self.absolute_extrude.set(false);
        Ok(())
    }

    fn cmd_g90(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        // Use absolute coordinates
        self.absolute_coord.set(true);
        Ok(())
    }

    fn cmd_g91(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        // Use relative coordinates
        self.absolute_coord.set(false);
        Ok(())
    }

    fn cmd_g92(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Set position
        let mut offsets = vec![];
        for a in ["X", "Y", "Z", "E"] {
            offsets.push(gcmd.get_float_opt(a, Lim::new())?);
        }
        for (i, offset) in offsets.iter().enumerate() {
            if let Some(mut offset) = *offset {
                if i == 3 {
                    offset *= self.extrude_factor.get();
                }
                let lp = self.last_position.borrow()[i];
                self.base_position.borrow_mut()[i] = lp - offset;
            }
        }
        if offsets.iter().all(|o| o.is_none()) {
            let lp = self.last_position.borrow()[..4].to_vec();
            self.base_position.borrow_mut()[..4].copy_from_slice(&lp);
        }
        Ok(())
    }

    fn cmd_m114(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Get Current Position
        let p = self.get_gcode_position();
        gcmd.respond_raw(&format!("X:{:.3} Y:{:.3} Z:{:.3} E:{:.3}", p[0], p[1], p[2], p[3]));
        Ok(())
    }

    fn cmd_m220(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Set speed factor override percentage
        let value = gcmd.get_float("S", Some(100.0), Lim::new().above(0.0))? / (60.0 * 100.0);
        self.speed.set(self.get_gcode_speed() * value);
        self.speed_factor.set(value);
        Ok(())
    }

    fn cmd_m221(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Set extrude factor override percentage
        let new_extrude_factor = gcmd.get_float("S", Some(100.0), Lim::new().above(0.0))? / 100.0;
        let last_e_pos = self.last_position.borrow()[3];
        let e_value = (last_e_pos - self.base_position.borrow()[3]) / self.extrude_factor.get();
        self.base_position.borrow_mut()[3] = last_e_pos - e_value * new_extrude_factor;
        self.extrude_factor.set(new_extrude_factor);
        Ok(())
    }

    pub fn cmd_set_gcode_offset(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mut move_delta = [0.0; 4];
        for (pos, axis) in ["X", "Y", "Z", "E"].iter().enumerate() {
            let offset = match gcmd.get_float_opt(axis, Lim::new())? {
                Some(o) => o,
                None => match gcmd.get_float_opt(&format!("{}_ADJUST", axis), Lim::new())? {
                    None => continue,
                    Some(o) => o + self.homing_position.borrow()[pos],
                },
            };
            let delta = offset - self.homing_position.borrow()[pos];
            move_delta[pos] = delta;
            self.base_position.borrow_mut()[pos] += delta;
            self.homing_position.borrow_mut()[pos] = offset;
        }
        // Move the toolhead the given offset if requested
        if gcmd.get_int("MOVE", Some(0), Lim::new())? != 0 {
            let speed = gcmd.get_float("MOVE_SPEED", Some(self.speed.get()), Lim::new().above(0.0))?;
            for (pos, delta) in move_delta.iter().enumerate() {
                self.last_position.borrow_mut()[pos] += delta;
            }
            let lp = self.last_position.borrow().clone();
            self.move_with_transform(&lp, speed)?;
        }
        Ok(())
    }

    fn cmd_save_gcode_state(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let state_name = gcmd.get_or("NAME", "default");
        let state = SavedState {
            absolute_coord: self.absolute_coord.get(),
            absolute_extrude: self.absolute_extrude.get(),
            base_position: self.base_position.borrow().clone(),
            last_position: self.last_position.borrow().clone(),
            homing_position: self.homing_position.borrow().clone(),
            speed: self.speed.get(),
            speed_factor: self.speed_factor.get(),
            extrude_factor: self.extrude_factor.get(),
        };
        self.saved_states.borrow_mut().insert(state_name, state);
        Ok(())
    }

    fn cmd_restore_gcode_state(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let state_name = gcmd.get_or("NAME", "default");
        let state = self.saved_states.borrow().get(&state_name).cloned();
        let Some(state) = state else {
            return Err(gcmd.error(format!("Unknown g-code state: {}", state_name)));
        };
        // Restore state
        self.absolute_coord.set(state.absolute_coord);
        self.absolute_extrude.set(state.absolute_extrude);
        self.base_position.borrow_mut()[..4].copy_from_slice(&state.base_position[..4]);
        *self.homing_position.borrow_mut() = state.homing_position.clone();
        self.speed.set(state.speed);
        self.speed_factor.set(state.speed_factor);
        self.extrude_factor.set(state.extrude_factor);
        // Restore the relative E position
        let e_diff = self.last_position.borrow()[3] - state.last_position[3];
        self.base_position.borrow_mut()[3] += e_diff;
        // Move the toolhead back if requested
        if gcmd.get_int("MOVE", Some(0), Lim::new())? != 0 {
            let speed = gcmd.get_float("MOVE_SPEED", Some(self.speed.get()), Lim::new().above(0.0))?;
            self.last_position.borrow_mut()[..3].copy_from_slice(&state.last_position[..3]);
            let lp = self.last_position.borrow().clone();
            self.move_with_transform(&lp, speed)?;
        }
        Ok(())
    }

    fn cmd_get_position(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let Ok(toolhead) = self.printer.lookup::<ToolHead>("toolhead") else {
            return Err(gcmd.error("Printer not ready"));
        };
        let kin = toolhead.get_kinematics();
        let steppers = kin.get_steppers();
        let mcu_pos: Vec<String> =
            steppers.iter().map(|s| format!("{}:{}", s.get_name(false), s.get_mcu_position(None))).collect();
        let cinfo: IndexMap<String, f64> =
            steppers.iter().map(|s| (s.get_name(false), s.get_commanded_position())).collect();
        let stepper_pos: Vec<String> = cinfo.iter().map(|(a, v)| format!("{}:{:.6}", a, v)).collect();
        let fmt = |names: &str, vals: &[f64]| -> String {
            names.chars().zip(vals).map(|(a, v)| format!("{}:{:.6}", a, v)).collect::<Vec<_>>().join(" ")
        };
        let kin_pos = fmt("XYZ", &kin.calc_position(&cinfo));
        let toolhead_pos = fmt("XYZE", &toolhead.get_position());
        let gcode_pos = fmt("XYZE", &self.last_position.borrow());
        let base_pos = fmt("XYZE", &self.base_position.borrow());
        let homing_pos = fmt("XYZ", &self.homing_position.borrow());
        gcmd.respond_info(
            &format!(
                "mcu: {}\nstepper: {}\nkinematic: {}\ntoolhead: {}\ngcode: {}\ngcode base: {}\ngcode homing: {}",
                mcu_pos.join(" "),
                stepper_pos.join(" "),
                kin_pos,
                toolhead_pos,
                gcode_pos,
                base_pos,
                homing_pos
            ),
            true,
        );
        Ok(())
    }
}

impl PrinterObject for GCodeMove {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let move_position = self.get_gcode_position();
        let mut d = IndexMap::new();
        d.insert("speed_factor".to_string(), PyVal::Float(self.get_gcode_speed_override()));
        d.insert("speed".to_string(), PyVal::Float(self.get_gcode_speed()));
        d.insert("extrude_factor".to_string(), PyVal::Float(self.extrude_factor.get()));
        d.insert("absolute_coordinates".to_string(), PyVal::Bool(self.absolute_coord.get()));
        d.insert("absolute_extrude".to_string(), PyVal::Bool(self.absolute_extrude.get()));
        d.insert("homing_origin".to_string(), coord_pyval(&self.homing_position.borrow()[..4]));
        d.insert("position".to_string(), coord_pyval(&self.last_position.borrow()[..4]));
        d.insert("gcode_position".to_string(), coord_pyval(&move_position));
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        *self.move_transform.borrow_mut() = None;
        *self.with_transform.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(GCodeMove::new(config)?)
}
