// Pause/Resume functionality with position capture/restore
// (port of klippy/extras/pause_resume.py)
//
// Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::virtual_sdcard::VirtualSD;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::webhooks::WebHooks;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const CMD_PAUSE_HELP: &str = "Pauses the current print";
const CMD_RESUME_HELP: &str = "Resumes the print from a pause";
const CMD_CLEAR_PAUSE_HELP: &str = "Clears the current paused state without resuming the print";
const CMD_CANCEL_PRINT_HELP: &str = "Cancel the current print";

pub struct PauseResume {
    printer: Rc<Printer>,
    gcode: Rc<GCodeDispatch>,
    recover_velocity: f64,
    v_sd: RefCell<Option<Rc<VirtualSD>>>,
    pub is_paused: Cell<bool>,
    sd_paused: Cell<bool>,
    pause_command_sent: Cell<bool>,
}

impl PauseResume {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let recover_velocity = config.getfloat("recover_velocity", 50.0, Lim::new())?;
        let pr = Rc::new(PauseResume {
            printer: printer.clone(),
            gcode: gcode.clone(),
            recover_velocity,
            v_sd: RefCell::new(None),
            is_paused: Cell::new(false),
            sd_paused: Cell::new(false),
            pause_command_sent: Cell::new(false),
        });
        let p = pr.clone();
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| {
                *p.v_sd.borrow_mut() = p.printer.lookup_object_opt("virtual_sdcard").and_then(downcast_rc::<VirtualSD>);
                Ok(())
            }),
        );
        type PCmd = fn(&PauseResume, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, PCmd, &str); 4] = [
            ("PAUSE", PauseResume::cmd_pause, CMD_PAUSE_HELP),
            ("RESUME", PauseResume::cmd_resume, CMD_RESUME_HELP),
            ("CLEAR_PAUSE", PauseResume::cmd_clear_pause, CMD_CLEAR_PAUSE_HELP),
            ("CANCEL_PRINT", PauseResume::cmd_cancel_print, CMD_CANCEL_PRINT_HELP),
        ];
        for (cmd, f, desc) in cmds {
            let p = pr.clone();
            gcode.register_command(cmd, Some(Rc::new(move |g| f(&p, g))), false, Some(desc))?;
        }
        let webhooks = printer.lookup::<WebHooks>("webhooks")?;
        for (path, script) in [("pause_resume/cancel", "CANCEL_PRINT"), ("pause_resume/pause", "PAUSE"), ("pause_resume/resume", "RESUME")] {
            let g = gcode.clone();
            webhooks.register_endpoint(path, Rc::new(move |_wr| g.run_script(script)))?;
        }
        Ok(pr)
    }

    fn v_sd(&self) -> KResult<Rc<VirtualSD>> {
        self.v_sd.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'do_resume'".into()))
    }

    pub fn is_sd_active(&self) -> bool {
        self.v_sd.borrow().as_ref().map(|v| v.is_active()).unwrap_or(false)
    }

    /// This sends the appropriate pause command from an event.  Note
    /// the difference between pause_command_sent and is_paused, the
    /// module isn't officially paused until the PAUSE gcode executes.
    pub fn send_pause_command(&self) -> KResult<()> {
        if !self.pause_command_sent.get() {
            if self.is_sd_active() {
                // Printing from virtual sd, run pause command
                self.sd_paused.set(true);
                self.v_sd()?.do_pause()?;
            } else {
                self.sd_paused.set(false);
                self.gcode.respond_info("action:paused", true);
            }
            self.pause_command_sent.set(true);
        }
        Ok(())
    }

    pub(crate) fn cmd_pause(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.is_paused.get() {
            gcmd.respond_info("Print already paused", true);
            return Ok(());
        }
        self.send_pause_command()?;
        self.gcode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE_STATE")?;
        self.is_paused.set(true);
        Ok(())
    }

    pub fn send_resume_command(&self) -> KResult<()> {
        if self.sd_paused.get() {
            // Printing from virtual sd, run pause command
            self.v_sd()?.do_resume()?;
            self.sd_paused.set(false);
        } else {
            self.gcode.respond_info("action:resumed", true);
        }
        self.pause_command_sent.set(false);
        Ok(())
    }

    pub(crate) fn cmd_resume(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if !self.is_paused.get() {
            gcmd.respond_info("Print is not paused, resume aborted", true);
            return Ok(());
        }
        let velocity = gcmd.get_float("VELOCITY", Some(self.recover_velocity), Lim::new())?;
        self.gcode.run_script_from_command(&format!("RESTORE_GCODE_STATE NAME=PAUSE_STATE MOVE=1 MOVE_SPEED={:.4}", velocity))?;
        self.send_resume_command()?;
        self.is_paused.set(false);
        Ok(())
    }

    fn cmd_clear_pause(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.is_paused.set(false);
        self.pause_command_sent.set(false);
        Ok(())
    }

    fn cmd_cancel_print(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.is_sd_active() || self.sd_paused.get() {
            self.v_sd()?.do_cancel()?;
        } else {
            gcmd.respond_info("action:cancel", true);
        }
        self.cmd_clear_pause(gcmd)
    }
}

impl PrinterObject for PauseResume {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("is_paused".to_string(), PyVal::Bool(self.is_paused.get()));
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        *self.v_sd.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PauseResume::new(config)?)
}
