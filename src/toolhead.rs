// Code for coordinating events on the printer toolhead
// (port of klippy/toolhead.py)
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Common suffixes: _d is distance (in mm), _v is velocity (in
//   mm/second), _v2 is velocity squared (mm^2/s^2), _t is time (in
//   seconds), _r is ratio (scalar between 0.0 and 1.0)
use crate::chelper::trapq::TrapQRef;
use crate::configfile::{ConfigWrapper, Lim, REQUIRED};
use crate::error::{KError, KResult};
use crate::extras::danger_options::get_danger_options;
use crate::extras::motion_queuing::PrinterMotionQueuing;
use crate::gcode::{coord_pyval, GCodeCommand, GCodeDispatch};
use crate::kinematics::{self, extruder as kinematics_extruder, Kinematics};
use crate::logging;
use crate::mcu::Mcu;
use crate::printer::{downcast_rc, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::reactor::{Reactor, Timer, NEVER};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

/// python max(a, b) / min(a, b) of floats (the first wins a tie or a nan)
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// min(list) / max(list)
fn py_min_of(vals: &[f64]) -> f64 {
    vals[1..].iter().fold(vals[0], |a, &b| py_min(a, b))
}

fn py_max_of(vals: &[f64]) -> f64 {
    vals[1..].iter().fold(vals[0], |a, &b| py_max(a, b))
}

/// an extra axis of the toolhead (extruders and the like); python duck types
pub trait ExtraAxis {
    fn check_move(&self, mv: &mut Move, ea_index: usize) -> KResult<()>;
    fn find_past_position(&self, print_time: f64) -> f64;
    fn calc_junction(&self, prev_move: &Move, mv: &Move, ea_index: usize) -> f64;
    fn process_move(&self, print_time: f64, mv: &Move, ea_index: usize) -> KResult<()>;
    fn get_name(&self) -> String;
    fn get_trapq(&self) -> Option<TrapQRef>;
    fn get_axis_gcode_id(&self) -> String;
    /// get_heater() of extruders (hasattr(extruder, "find_past_position")
    /// callers); other axes have none
    fn get_heater(&self) -> KResult<Rc<crate::extras::heaters::Heater>> {
        Err(KError::Other("object has no attribute 'get_heater'".into()))
    }
    fn as_any(&self) -> &dyn Any;
}

pub type ExtraAxisRef = Rc<dyn ExtraAxis>;

/// a move timing callback: cb(next_move_time)
pub type TimingCallback = Rc<dyn Fn(f64) -> KResult<()>>;

// Class to track each move request
pub struct Move {
    pub start_pos: Vec<f64>,
    pub end_pos: Vec<f64>,
    pub accel: f64,
    pub junction_deviation: f64,
    pub timing_callbacks: Vec<TimingCallback>,
    pub is_kinematic_move: bool,
    pub axes_d: Vec<f64>,
    pub move_d: f64,
    pub axes_r: Vec<f64>,
    pub min_move_t: f64,
    pub max_start_v2: f64,
    pub max_cruise_v2: f64,
    pub delta_v2: f64,
    pub max_smoothed_v2: f64,
    pub smooth_delta_v2: f64,
    pub next_junction_v2: f64,
    pub start_v: f64,
    pub cruise_v: f64,
    pub end_v: f64,
    pub accel_t: f64,
    pub cruise_t: f64,
    pub decel_t: f64,
}

impl Move {
    fn new(toolhead: &ToolHead, start_pos: &[f64], end_pos: &[f64], speed: f64) -> Move {
        let mut accel = toolhead.max_accel.get();
        let mut velocity = py_min(speed, toolhead.max_velocity.get());
        let mut is_kinematic_move = true;
        let mut axes_d: Vec<f64> = start_pos.iter().zip(end_pos).map(|(sp, ep)| ep - sp).collect();
        let mut end_pos = end_pos.to_vec();
        let mut move_d = crate::pyfmt::py_sum(axes_d[..3].iter().map(|d| d * d)).sqrt();
        let inv_move_d;
        if move_d < 0.000000001 {
            // Extrude only move
            end_pos[..3].copy_from_slice(&start_pos[..3]);
            axes_d[0] = 0.0;
            axes_d[1] = 0.0;
            axes_d[2] = 0.0;
            let abs_d: Vec<f64> = axes_d[3..].iter().map(|ad| ad.abs()).collect();
            // max() of an empty list raises ValueError in python
            move_d = if abs_d.is_empty() { 0.0 } else { py_max_of(&abs_d) };
            inv_move_d = if move_d != 0.0 { 1.0 / move_d } else { 0.0 };
            accel = 99999999.9;
            velocity = speed;
            is_kinematic_move = false;
        } else {
            inv_move_d = 1.0 / move_d;
        }
        let axes_r = axes_d.iter().map(|d| d * inv_move_d).collect();
        Move {
            start_pos: start_pos.to_vec(),
            end_pos,
            accel,
            junction_deviation: toolhead.junction_deviation.get(),
            timing_callbacks: vec![],
            is_kinematic_move,
            axes_d,
            move_d,
            axes_r,
            min_move_t: move_d / velocity,
            // Junction speeds are tracked in velocity squared.  The
            // delta_v2 is the maximum amount of this squared-velocity that
            // can change in this move.
            max_start_v2: 0.0,
            max_cruise_v2: velocity * velocity,
            delta_v2: 2.0 * move_d * accel,
            max_smoothed_v2: 0.0,
            smooth_delta_v2: 2.0 * move_d * toolhead.max_accel_to_decel.get(),
            next_junction_v2: 999999999.9,
            start_v: 0.0,
            cruise_v: 0.0,
            end_v: 0.0,
            accel_t: 0.0,
            cruise_t: 0.0,
            decel_t: 0.0,
        }
    }

    pub fn limit_speed(&mut self, speed: f64, accel: f64) {
        let speed2 = speed * speed;
        if speed2 < self.max_cruise_v2 {
            self.max_cruise_v2 = speed2;
            self.min_move_t = self.move_d / speed;
        }
        self.accel = py_min(self.accel, accel);
        self.delta_v2 = 2.0 * self.move_d * self.accel;
        self.smooth_delta_v2 = py_min(self.smooth_delta_v2, self.delta_v2);
    }

    pub fn limit_next_junction_speed(&mut self, speed: f64) {
        self.next_junction_v2 = py_min(self.next_junction_v2, speed * speed);
    }

    pub fn move_error(&self, msg: &str) -> KError {
        let ep = &self.end_pos;
        KError::command(format!("{}: {:.3} {:.3} {:.3} [{:.3}]", msg, ep[0], ep[1], ep[2], ep[3]))
    }

    fn calc_junction(&mut self, prev_move: &Move, extra_axes: &[ExtraAxisRef]) {
        if !self.is_kinematic_move || !prev_move.is_kinematic_move {
            return;
        }
        // Allow extra axes to calculate maximum junction
        let mut limits = vec![
            self.max_cruise_v2,
            prev_move.max_cruise_v2,
            prev_move.next_junction_v2,
            prev_move.max_start_v2 + prev_move.delta_v2,
        ];
        for (e_index, ea) in extra_axes.iter().enumerate() {
            limits.push(ea.calc_junction(prev_move, self, e_index + 3));
        }
        let mut max_start_v2 = py_min_of(&limits);
        // Find max velocity using "approximated centripetal velocity"
        let axes_r = &self.axes_r;
        let prev_axes_r = &prev_move.axes_r;
        let junction_cos_theta = -(axes_r[0] * prev_axes_r[0] + axes_r[1] * prev_axes_r[1] + axes_r[2] * prev_axes_r[2]);
        let sin_theta_d2 = py_max(0.5 * (1.0 - junction_cos_theta), 0.0).sqrt();
        let cos_theta_d2 = py_max(0.5 * (1.0 + junction_cos_theta), 0.0).sqrt();
        let one_minus_sin_theta_d2 = 1.0 - sin_theta_d2;
        if one_minus_sin_theta_d2 > 0.0 && cos_theta_d2 > 0.0 {
            let r_jd = sin_theta_d2 / one_minus_sin_theta_d2;
            let move_jd_v2 = r_jd * self.junction_deviation * self.accel;
            let pmove_jd_v2 = r_jd * prev_move.junction_deviation * prev_move.accel;
            // Approximated circle must contact moves no further than mid-move
            //   centripetal_v2 = .5 * self.move_d * self.accel * tan_theta_d2
            let quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2;
            let move_centripetal_v2 = self.delta_v2 * quarter_tan_theta_d2;
            let pmove_centripetal_v2 = prev_move.delta_v2 * quarter_tan_theta_d2;
            max_start_v2 =
                py_min_of(&[max_start_v2, move_jd_v2, pmove_jd_v2, move_centripetal_v2, pmove_centripetal_v2]);
        }
        // Apply limits
        self.max_start_v2 = max_start_v2;
        self.max_smoothed_v2 = py_min(max_start_v2, prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2);
    }

    fn set_junction(&mut self, start_v2: f64, cruise_v2: f64, end_v2: f64) {
        // Determine accel, cruise, and decel portions of the move distance
        let half_inv_accel = 0.5 / self.accel;
        let accel_d = (cruise_v2 - start_v2) * half_inv_accel;
        let decel_d = (cruise_v2 - end_v2) * half_inv_accel;
        let cruise_d = self.move_d - accel_d - decel_d;
        // Determine move velocities
        let start_v = start_v2.sqrt();
        let cruise_v = cruise_v2.sqrt();
        let end_v = end_v2.sqrt();
        self.start_v = start_v;
        self.cruise_v = cruise_v;
        self.end_v = end_v;
        // Determine time spent in each portion of move (time is the
        // distance divided by average velocity)
        self.accel_t = accel_d / ((start_v + cruise_v) * 0.5);
        self.cruise_t = cruise_d / cruise_v;
        self.decel_t = decel_d / ((end_v + cruise_v) * 0.5);
    }
}

pub const LOOKAHEAD_FLUSH_TIME: f64 = 0.250;

// Class to track a list of pending move requests and to facilitate
// "look-ahead" across moves to reduce acceleration between moves.
pub struct LookAheadQueue {
    pub queue: Vec<Move>,
    junction_flush: f64,
}

impl LookAheadQueue {
    fn new() -> Self {
        LookAheadQueue { queue: vec![], junction_flush: LOOKAHEAD_FLUSH_TIME }
    }

    pub fn reset(&mut self) {
        self.queue.clear();
        self.junction_flush = LOOKAHEAD_FLUSH_TIME;
    }

    pub fn set_flush_time(&mut self, flush_time: f64) {
        self.junction_flush = flush_time;
    }

    pub fn get_last(&mut self) -> Option<&mut Move> {
        self.queue.last_mut()
    }

    pub fn flush(&mut self, lazy: bool) -> Vec<Move> {
        self.junction_flush = LOOKAHEAD_FLUSH_TIME;
        let mut update_flush_count = lazy;
        let queue = &mut self.queue;
        let mut flush_count = queue.len();
        // Traverse queue from last to first move and determine maximum
        // junction speed assuming the robot comes to a complete stop
        // after the last move.
        let mut delayed: Vec<(usize, f64, f64)> = vec![];
        let (mut next_end_v2, mut next_smoothed_v2, mut peak_cruise_v2) = (0.0, 0.0, 0.0);
        for i in (0..flush_count).rev() {
            let mv = &queue[i];
            let reachable_start_v2 = next_end_v2 + mv.delta_v2;
            let start_v2 = py_min(mv.max_start_v2, reachable_start_v2);
            let reachable_smoothed_v2 = next_smoothed_v2 + mv.smooth_delta_v2;
            let smoothed_v2 = py_min(mv.max_smoothed_v2, reachable_smoothed_v2);
            if smoothed_v2 < reachable_smoothed_v2 {
                // It's possible for this move to accelerate
                if smoothed_v2 + mv.smooth_delta_v2 > next_smoothed_v2 || !delayed.is_empty() {
                    // This move can decelerate or this is a full accel
                    // move after a full decel move
                    if update_flush_count && peak_cruise_v2 != 0.0 {
                        flush_count = i;
                        update_flush_count = false;
                    }
                    peak_cruise_v2 = py_min(mv.max_cruise_v2, (smoothed_v2 + reachable_smoothed_v2) * 0.5);
                    if !delayed.is_empty() {
                        // Propagate peak_cruise_v2 to any delayed moves
                        if !update_flush_count && i < flush_count {
                            let mut mc_v2 = peak_cruise_v2;
                            for &(m, ms_v2, me_v2) in delayed.iter().rev() {
                                mc_v2 = py_min(mc_v2, ms_v2);
                                queue[m].set_junction(py_min(ms_v2, mc_v2), mc_v2, py_min(me_v2, mc_v2));
                            }
                        }
                        delayed.clear();
                    }
                }
                if !update_flush_count && i < flush_count {
                    let mv = &mut queue[i];
                    let cruise_v2 =
                        py_min_of(&[(start_v2 + reachable_start_v2) * 0.5, mv.max_cruise_v2, peak_cruise_v2]);
                    mv.set_junction(py_min(start_v2, cruise_v2), cruise_v2, py_min(next_end_v2, cruise_v2));
                }
            } else {
                // Delay calculating this move until peak_cruise_v2 is known
                delayed.push((i, start_v2, next_end_v2));
            }
            next_end_v2 = start_v2;
            next_smoothed_v2 = smoothed_v2;
        }
        if update_flush_count || flush_count == 0 {
            return vec![];
        }
        // Remove processed moves from the queue
        queue.drain(..flush_count).collect()
    }

    /// add_move(); Some(want_flush), None for the first move
    pub fn add_move(&mut self, mut mv: Move, extra_axes: &[ExtraAxisRef]) -> bool {
        if let Some(prev) = self.queue.last() {
            mv.calc_junction(prev, extra_axes);
            self.junction_flush -= mv.min_move_t;
            self.queue.push(mv);
            // Check if enough moves have been queued to reach the target flush time.
            return self.junction_flush <= 0.0;
        }
        self.queue.push(mv);
        false
    }
}

pub const BUFFER_TIME_HIGH: f64 = 2.0;
pub const BUFFER_TIME_START: f64 = 0.250;

// Main code to track events (and their timing) on the printer toolhead
pub struct ToolHead {
    me: std::rc::Weak<ToolHead>,
    printer: Rc<Printer>,
    reactor: Rc<Reactor>,
    pub(crate) mcu: Rc<Mcu>,
    pub lookahead: RefCell<LookAheadQueue>,
    commanded_pos: RefCell<Vec<f64>>,
    // Velocity and acceleration control
    pub max_velocity: Cell<f64>,
    pub max_accel: Cell<f64>,
    pub min_cruise_ratio: Cell<f64>,
    pub square_corner_velocity: Cell<f64>,
    pub orig_cfg: RefCell<IndexMap<String, f64>>,
    pub junction_deviation: Cell<f64>,
    pub max_accel_to_decel: Cell<f64>,
    // Input stall detection
    pub(crate) check_stall_time: Cell<f64>,
    print_stall: Cell<i64>,
    // Input pause tracking
    pub(crate) can_pause: Cell<bool>,
    pub(crate) need_check_pause: Cell<f64>,
    // Print time tracking
    pub(crate) print_time: Cell<f64>,
    pub(crate) special_queuing_state: RefCell<&'static str>,
    priming_timer: RefCell<Option<Timer>>,
    // Setup for generating moves
    motion_queuing: Rc<PrinterMotionQueuing>,
    trapq: TrapQRef,
    extra_axes: RefCell<Vec<ExtraAxisRef>>,
    kin: RefCell<Option<Rc<dyn Kinematics>>>,
}

/// the velocity limits ToolHead.__init__ reads
pub struct ToolHeadLimits {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub min_cruise_ratio: f64,
    pub square_corner_velocity: f64,
}

impl ToolHead {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<ToolHead>> {
        // Velocity and acceleration control
        let max_velocity = config.getfloat("max_velocity", REQUIRED, Lim::new().above(0.0))?;
        let max_accel = config.getfloat("max_accel", REQUIRED, Lim::new().above(0.0))?;
        let min_cruise_ratio = config.getfloat("minimum_cruise_ratio", 0.5, Lim::new().below(1.0).min(0.0))?;
        let square_corner_velocity = config.getfloat("square_corner_velocity", 5.0, Lim::new().min(0.0))?;
        let limits = ToolHeadLimits { max_velocity, max_accel, min_cruise_ratio, square_corner_velocity };
        ToolHead::new_with(config, limits, &|th| {
            // Create kinematics class
            let kin_name = config.get("kinematics", REQUIRED)?;
            let load = kinematics::KINEMATICS.iter().find(|(n, _)| *n == kin_name).map(|(_, l)| *l);
            let kin = match load {
                Some(Some(load)) => load(th, config)?,
                Some(None) => {
                    // ponytail: an unported kinematics fails like a load failure
                    let msg = format!("Error loading kinematics '{}'", kin_name);
                    logging::exception(&msg);
                    return Err(KError::config(format!("{} (not ported to Kaliru yet)", msg)));
                }
                None => {
                    // ModuleNotFoundError from importlib (of the first missing
                    // dotted component)
                    let module = format!("klippy.kinematics.{}", kin_name.split('.').next().unwrap_or(""));
                    let cause = KError::Py("ModuleNotFoundError".into(), format!("No module named {}", crate::pyfmt::py_str_repr(&module)));
                    let msg = format!("Error loading kinematics '{}'", kin_name);
                    logging::exception_err(&msg, &cause);
                    return Err(KError::config(msg).with_context(cause));
                }
            };
            if config.has_section("dual_carriage") && !kin.supports_dual_carriage() {
                return Err(KError::config(format!("dual_carriage not compatible with '{}' kinematics system", kin_name)));
            }
            for name in ["max_x_velocity", "max_x_accel", "max_y_velocity", "max_y_accel", "max_z_velocity", "max_z_accel"] {
                if let Some(v) = kin.limit(name) {
                    th.orig_cfg.borrow_mut().insert(name.to_string(), v.get());
                }
            }
            Ok(kin)
        })
    }

    /// ToolHead.__init__ after the velocity limits are read: `load_kin`
    /// creates the kinematics (trad_rack's TradRackToolHead subclass
    /// supplies its own limits and kinematics)
    pub fn new_with(
        config: &ConfigWrapper,
        limits: ToolHeadLimits,
        load_kin: &dyn Fn(&Rc<ToolHead>) -> KResult<Rc<dyn Kinematics>>,
    ) -> KResult<Rc<ToolHead>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let mcu = printer.lookup::<Mcu>("mcu")?;
        let ToolHeadLimits { max_velocity, max_accel, min_cruise_ratio, square_corner_velocity } = limits;
        let mut orig_cfg = IndexMap::new();
        orig_cfg.insert("max_velocity".to_string(), max_velocity);
        orig_cfg.insert("max_accel".to_string(), max_accel);
        orig_cfg.insert("min_cruise_ratio".to_string(), min_cruise_ratio);
        orig_cfg.insert("square_corner_velocity".to_string(), square_corner_velocity);
        // Setup for generating moves
        let motion_queuing = printer
            .load_object(config, "motion_queuing", true)?
            .and_then(downcast_rc::<PrinterMotionQueuing>)
            .ok_or_else(|| KError::Other("motion_queuing has an unexpected type".into()))?;
        let trapq = motion_queuing.allocate_trapq();
        let mut lookahead = LookAheadQueue::new();
        lookahead.set_flush_time(BUFFER_TIME_HIGH);
        let can_pause = !mcu.is_fileoutput();
        let th = Rc::new_cyclic(|me| ToolHead {
            me: me.clone(),
            printer: printer.clone(),
            reactor,
            mcu,
            lookahead: RefCell::new(lookahead),
            commanded_pos: RefCell::new(vec![0.0, 0.0, 0.0, 0.0]),
            max_velocity: Cell::new(max_velocity),
            max_accel: Cell::new(max_accel),
            min_cruise_ratio: Cell::new(min_cruise_ratio),
            square_corner_velocity: Cell::new(square_corner_velocity),
            orig_cfg: RefCell::new(orig_cfg),
            junction_deviation: Cell::new(0.0),
            max_accel_to_decel: Cell::new(0.0),
            check_stall_time: Cell::new(0.0),
            print_stall: Cell::new(0),
            can_pause: Cell::new(can_pause),
            need_check_pause: Cell::new(-1.0),
            print_time: Cell::new(0.0),
            special_queuing_state: RefCell::new("NeedPrime"),
            priming_timer: RefCell::new(None),
            motion_queuing: motion_queuing.clone(),
            trapq,
            extra_axes: RefCell::new(vec![]),
            kin: RefCell::new(None),
        });
        th.calc_junction_deviation();
        let t = th.clone();
        motion_queuing.register_flush_callback(Rc::new(move |ft, sgt| t.handle_step_flush(ft, sgt)), true);
        // Create kinematics class
        let extruder: ExtraAxisRef = Rc::new(kinematics_extruder::DummyExtruder::new(&printer));
        th.extra_axes.borrow_mut().push(extruder);
        let kin = load_kin(&th)?;
        *th.kin.borrow_mut() = Some(kin);

        // Register handlers
        let t = th.clone();
        printer.register_event_handler(
            "klippy:shutdown",
            Rc::new(move |_| {
                t.handle_shutdown();
                Ok(())
            }),
        );
        Ok(th)
    }

    /// lookahead.get_last().min_move_t
    pub fn last_move_min_move_t(&self) -> Option<f64> {
        self.lookahead.borrow_mut().get_last().map(|m| m.min_move_t)
    }

    fn kin(&self) -> Rc<dyn Kinematics> {
        self.kin.borrow().clone().expect("kinematics")
    }

    pub fn get_active_rails_for_axis(&self, axis: char) -> Vec<crate::stepper::RailRef> {
        // axis is 'x,y,z'
        let mut active_rails = vec![];
        for rail in self.kin().rails() {
            if rail.get_steppers().iter().any(|s| s.is_active_axis(axis)) {
                active_rails.push(rail);
            }
        }
        active_rails
    }

    // Print time tracking
    fn advance_move_time(&self, next_print_time: f64) {
        self.print_time.set(py_max(self.print_time.get(), next_print_time));
    }

    fn calc_print_time(&self) -> KResult<()> {
        let curtime = self.reactor.monotonic();
        let est_print_time = self.mcu.estimated_print_time(curtime);
        let kin_time = self.motion_queuing.calc_step_gen_restart(est_print_time);
        let min_print_time = py_max(est_print_time + BUFFER_TIME_START, kin_time);
        if min_print_time > self.print_time.get() {
            self.print_time.set(min_print_time);
            let print_time = self.print_time.get();
            self.printer.send_event("toolhead:sync_print_time", &[&curtime, &est_print_time, &print_time])?;
        }
        Ok(())
    }

    fn process_lookahead(&self, lazy: bool) -> KResult<()> {
        let moves = self.lookahead.borrow_mut().flush(lazy);
        if moves.is_empty() {
            return Ok(());
        }
        // Resync print_time if necessary
        if !self.special_queuing_state.borrow().is_empty() {
            // Transition from "NeedPrime"/"Priming" state to main state
            *self.special_queuing_state.borrow_mut() = "";
            self.need_check_pause.set(-1.0);
            self.calc_print_time()?;
        }
        // Queue moves into trapezoid motion queue (trapq)
        let mut next_move_time = self.print_time.get();
        {
            let _guard = self.reactor.assert_no_pause();
            let extra_axes = self.extra_axes.borrow().clone();
            for mv in &moves {
                if mv.is_kinematic_move {
                    self.trapq.borrow_mut().append(
                        next_move_time,
                        mv.accel_t,
                        mv.cruise_t,
                        mv.decel_t,
                        mv.start_pos[0],
                        mv.start_pos[1],
                        mv.start_pos[2],
                        mv.axes_r[0],
                        mv.axes_r[1],
                        mv.axes_r[2],
                        mv.start_v,
                        mv.cruise_v,
                        mv.accel,
                    );
                }
                for (e_index, ea) in extra_axes.iter().enumerate() {
                    if mv.axes_d[e_index + 3] != 0.0 {
                        ea.process_move(next_move_time, mv, e_index + 3)?;
                    }
                }
                next_move_time = next_move_time + mv.accel_t + mv.cruise_t + mv.decel_t;
                for cb in &mv.timing_callbacks {
                    cb(next_move_time)?;
                }
            }
        }
        // Generate steps for moves
        self.advance_move_time(next_move_time);
        self.motion_queuing.note_mcu_movequeue_activity(next_move_time, true);
        Ok(())
    }

    fn flush_lookahead(&self, is_runout: bool) -> KResult<()> {
        // Transit from "NeedPrime"/"Priming"/main state to "NeedPrime"
        let prev_print_time = self.print_time.get();
        self.process_lookahead(false)?;
        *self.special_queuing_state.borrow_mut() = "NeedPrime";
        self.need_check_pause.set(-1.0);
        self.lookahead.borrow_mut().set_flush_time(BUFFER_TIME_HIGH);
        self.check_stall_time.set(0.0);
        if is_runout && prev_print_time != self.print_time.get() {
            self.check_stall_time.set(self.print_time.get());
        }
        Ok(())
    }

    pub fn flush_step_generation(&self) -> KResult<()> {
        self.flush_lookahead(false)?;
        self.motion_queuing.flush_all_steps()
    }

    pub fn get_last_move_time(&self) -> KResult<f64> {
        if !self.special_queuing_state.borrow().is_empty() {
            self.flush_lookahead(false)?;
            self.calc_print_time()?;
        } else {
            self.process_lookahead(false)?;
        }
        Ok(self.print_time.get())
    }

    fn check_pause(&self) -> KResult<()> {
        let mut eventtime = self.reactor.monotonic();
        let mut est_print_time = self.mcu.estimated_print_time(eventtime);
        let mut buffer_time = self.print_time.get() - est_print_time;
        if !self.special_queuing_state.borrow().is_empty() {
            if self.check_stall_time.get() != 0.0 {
                // Was in "NeedPrime" state and got there from idle input
                if est_print_time < self.check_stall_time.get() {
                    self.print_stall.set(self.print_stall.get() + 1);
                }
                self.check_stall_time.set(0.0);
            }
            // Transition from "NeedPrime"/"Priming" state to "Priming" state
            *self.special_queuing_state.borrow_mut() = "Priming";
            self.need_check_pause.set(-1.0);
            if self.priming_timer.borrow().is_none() {
                let w = self.me.clone();
                let timer = self.reactor.register_timer(
                    Box::new(move |eventtime| match w.upgrade() {
                        Some(th) => th.priming_handler(eventtime),
                        None => NEVER,
                    }),
                    NEVER,
                );
                *self.priming_timer.borrow_mut() = Some(timer);
            }
            let wtime = eventtime + py_max(0.100, buffer_time - BUFFER_TIME_HIGH);
            if let Some(t) = self.priming_timer.borrow().as_ref() {
                self.reactor.update_timer(t, wtime);
            }
        }
        // Check if there are lots of queued moves and pause if so
        loop {
            let pause_time = buffer_time - BUFFER_TIME_HIGH;
            if pause_time <= 0.0 {
                break;
            }
            if !self.can_pause.get() {
                self.need_check_pause.set(NEVER);
                return Ok(());
            }
            eventtime = self.reactor.pause(eventtime + py_min(1.0, pause_time))?;
            est_print_time = self.mcu.estimated_print_time(eventtime);
            buffer_time = self.print_time.get() - est_print_time;
        }
        if self.special_queuing_state.borrow().is_empty() {
            // In main state - defer pause checking until needed
            self.need_check_pause.set(est_print_time + BUFFER_TIME_HIGH + 0.100);
        }
        Ok(())
    }

    fn priming_handler(&self, _eventtime: f64) -> f64 {
        if let Some(t) = self.priming_timer.borrow_mut().take() {
            self.reactor.unregister_timer(&t);
        }
        let res = if *self.special_queuing_state.borrow() == "Priming" { self.flush_lookahead(true) } else { Ok(()) };
        if res.is_err() {
            logging::exception("Exception in priming_handler");
            self.printer.invoke_shutdown("Exception in priming_handler", None);
        }
        NEVER
    }

    fn handle_step_flush(&self, _flush_time: f64, step_gen_time: f64) -> KResult<()> {
        if !self.special_queuing_state.borrow().is_empty() {
            return Ok(());
        }
        // In "main" state - flush lookahead if buffer runs low
        let kin_flush_delay = self.motion_queuing.get_kin_flush_delay();
        if step_gen_time >= self.print_time.get() - kin_flush_delay {
            self.flush_lookahead(true)?;
        }
        Ok(())
    }

    // Movement commands
    pub fn get_position(&self) -> Vec<f64> {
        self.commanded_pos.borrow().clone()
    }

    pub fn set_position(&self, newpos: &[f64], homing_axes: &str) -> KResult<()> {
        self.flush_step_generation()?;
        self.trapq.borrow_mut().set_position(self.print_time.get(), newpos[0], newpos[1], newpos[2]);
        self.commanded_pos.borrow_mut()[..3].copy_from_slice(&newpos[..3]);
        self.kin().set_position(newpos, homing_axes)?;
        self.printer.send_event("toolhead:set_position", &[])
    }

    pub fn limit_next_junction_speed(&self, speed: f64) {
        if let Some(last_move) = self.lookahead.borrow_mut().get_last() {
            last_move.limit_next_junction_speed(speed);
        }
    }

    pub fn move_to(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        let commanded_pos = self.commanded_pos.borrow().clone();
        let mut mv = Move::new(self, &commanded_pos, newpos, speed);
        if mv.move_d == 0.0 {
            return Ok(());
        }
        if mv.is_kinematic_move {
            self.kin().check_move(&mut mv)?;
        }
        let extra_axes = self.extra_axes.borrow().clone();
        for (e_index, ea) in extra_axes.iter().enumerate() {
            if mv.axes_d[e_index + 3] != 0.0 {
                ea.check_move(&mut mv, e_index + 3)?;
            }
        }
        *self.commanded_pos.borrow_mut() = mv.end_pos.clone();
        let want_flush = self.lookahead.borrow_mut().add_move(mv, &extra_axes);
        if want_flush {
            self.process_lookahead(true)?;
        }
        if self.print_time.get() > self.need_check_pause.get() {
            self.check_pause()?;
        }
        Ok(())
    }

    pub fn manual_move(&self, coord: &[Option<f64>], speed: f64) -> KResult<()> {
        let mut curpos = self.commanded_pos.borrow().clone();
        for (i, c) in coord.iter().enumerate() {
            if let Some(c) = c {
                curpos[i] = *c;
            }
        }
        self.move_to(&curpos, speed)?;
        self.printer.send_event("toolhead:manual_move", &[])
    }

    pub fn dwell(&self, delay: f64) -> KResult<()> {
        let next_print_time = self.get_last_move_time()? + py_max(0.0, delay);
        self.advance_move_time(next_print_time);
        self.check_pause()
    }

    pub fn wait_moves(&self) -> KResult<()> {
        self.flush_lookahead(false)?;
        let mut eventtime = self.reactor.monotonic();
        while self.special_queuing_state.borrow().is_empty()
            || self.print_time.get() >= self.mcu.estimated_print_time(eventtime)
        {
            if !self.can_pause.get() {
                // In batch mode the background flush intentionally lags;
                // generate all pending steps so that stepper positions
                // are deterministic after a wait_moves() (eg, M400)
                self.motion_queuing.flush_all_steps()?;
                break;
            }
            eventtime = self.reactor.pause(eventtime + 0.100)?;
        }
        Ok(())
    }

    pub fn set_extruder(&self, extruder: ExtraAxisRef, extrude_pos: f64) {
        // XXX - should use add_extra_axis
        self.extra_axes.borrow_mut()[0] = extruder;
        self.commanded_pos.borrow_mut()[3] = extrude_pos;
    }

    pub fn get_extruder(&self) -> ExtraAxisRef {
        self.extra_axes.borrow()[0].clone()
    }

    pub fn add_extra_axis(&self, ea: ExtraAxisRef, axis_pos: f64) -> KResult<()> {
        self.flush_lookahead(false)?;
        self.extra_axes.borrow_mut().push(ea);
        self.commanded_pos.borrow_mut().push(axis_pos);
        self.printer.send_event("toolhead:update_extra_axes", &[])
    }

    pub fn remove_extra_axis(&self, ea: &ExtraAxisRef) -> KResult<()> {
        self.flush_lookahead(false)?;
        let pos = self.extra_axes.borrow().iter().position(|e| Rc::ptr_eq(e, ea));
        let Some(pos) = pos else { return Ok(()) };
        self.commanded_pos.borrow_mut().remove(pos + 3);
        self.extra_axes.borrow_mut().remove(pos);
        self.printer.send_event("toolhead:update_extra_axes", &[])
    }

    /// [None, None, None] + self.extra_axes
    pub fn get_extra_axes(&self) -> Vec<Option<ExtraAxisRef>> {
        let mut res = vec![None, None, None];
        res.extend(self.extra_axes.borrow().iter().cloned().map(Some));
        res
    }

    // Homing "drip move" handling

    fn drip_load_trapq(&self, submit_move: Move) -> (f64, f64) {
        // Queue move into trapezoid motion queue (trapq)
        let extra_axes = self.extra_axes.borrow().clone();
        if submit_move.move_d != 0.0 {
            *self.commanded_pos.borrow_mut() = submit_move.end_pos.clone();
            self.lookahead.borrow_mut().add_move(submit_move, &extra_axes);
        }
        let moves = self.lookahead.borrow_mut().flush(false);
        // _calc_print_time() errors come from event handlers
        let _ = self.calc_print_time();
        let start_time = self.print_time.get();
        let mut end_time = start_time;
        for mv in &moves {
            self.trapq.borrow_mut().append(
                end_time,
                mv.accel_t,
                mv.cruise_t,
                mv.decel_t,
                mv.start_pos[0],
                mv.start_pos[1],
                mv.start_pos[2],
                mv.axes_r[0],
                mv.axes_r[1],
                mv.axes_r[2],
                mv.start_v,
                mv.cruise_v,
                mv.accel,
            );
            end_time = end_time + mv.accel_t + mv.cruise_t + mv.decel_t;
        }
        self.lookahead.borrow_mut().reset();
        (start_time, end_time)
    }

    pub fn drip_move(&self, newpos: &[f64], speed: f64, drip_completion: &Rc<crate::reactor::ReactorCompletion<bool>>) -> KResult<()> {
        // Create and verify move is valid
        let commanded_pos = self.commanded_pos.borrow().clone();
        let mut newpos: Vec<f64> = newpos[..3].to_vec();
        newpos.extend_from_slice(&commanded_pos[3..]);
        let mut mv = Move::new(self, &commanded_pos, &newpos, speed);
        if mv.move_d != 0.0 {
            self.kin().check_move(&mut mv)?;
        }
        // Make sure stepper movement doesn't start before nominal start time
        let kin_flush_delay = self.motion_queuing.get_kin_flush_delay();
        self.dwell(kin_flush_delay)?;
        // Transmit move in "drip" mode
        self.process_lookahead(false)?;
        let (start_time, end_time) = self.drip_load_trapq(mv);
        self.motion_queuing.drip_update_time(start_time, end_time, drip_completion)?;
        // Move finished; cleanup any remnants on trapq
        self.motion_queuing.wipe_trapq(&self.trapq);
        Ok(())
    }

    // Misc commands
    pub fn check_busy(&self, eventtime: f64) -> (f64, f64, bool) {
        let est_print_time = self.mcu.estimated_print_time(eventtime);
        let lookahead_empty = self.lookahead.borrow().queue.is_empty();
        (self.print_time.get(), est_print_time, lookahead_empty)
    }

    fn handle_shutdown(&self) {
        self.can_pause.set(false);
        self.lookahead.borrow_mut().reset();
    }

    pub fn get_kinematics(&self) -> Rc<dyn Kinematics> {
        self.kin()
    }

    pub fn get_trapq(&self) -> TrapQRef {
        self.trapq.clone()
    }

    pub fn register_lookahead_callback(&self, callback: TimingCallback) -> KResult<()> {
        {
            let mut lookahead = self.lookahead.borrow_mut();
            if let Some(last_move) = lookahead.get_last() {
                last_move.timing_callbacks.push(callback);
                return Ok(());
            }
        }
        callback(self.get_last_move_time()?)
    }

    pub fn get_max_velocity(&self) -> (f64, f64) {
        (self.max_velocity.get(), self.max_accel.get())
    }

    fn calc_junction_deviation(&self) {
        let scv2 = self.square_corner_velocity.get() * self.square_corner_velocity.get();
        self.junction_deviation.set(scv2 * (2.0f64.sqrt() - 1.0) / self.max_accel.get());
        self.max_accel_to_decel.set(self.max_accel.get() * (1.0 - self.min_cruise_ratio.get()));
    }

    pub fn set_max_velocities(
        &self,
        max_velocity: Option<f64>,
        max_accel: Option<f64>,
        square_corner_velocity: Option<f64>,
        min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        if let Some(v) = max_velocity {
            self.max_velocity.set(v);
        }
        if let Some(v) = max_accel {
            self.max_accel.set(v);
        }
        if let Some(v) = square_corner_velocity {
            self.square_corner_velocity.set(v);
        }
        if let Some(v) = min_cruise_ratio {
            self.min_cruise_ratio.set(v);
        }
        self.calc_junction_deviation();
        (self.max_velocity.get(), self.max_accel.get(), self.square_corner_velocity.get(), self.min_cruise_ratio.get())
    }

    pub fn set_accel(&self, accel: f64) {
        self.max_accel.set(accel);
        self.calc_junction_deviation();
    }
}

impl PrinterObject for ToolHead {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        let print_time = self.print_time.get();
        let estimated_print_time = self.mcu.estimated_print_time(eventtime);
        let extruder = self.extra_axes.borrow()[0].clone();
        let mut res = self.kin().get_status(eventtime);
        let commanded_pos = self.commanded_pos.borrow();
        res.insert("print_time".into(), PyVal::Float(print_time));
        res.insert("stalls".into(), PyVal::Int(self.print_stall.get()));
        res.insert("estimated_print_time".into(), PyVal::Float(estimated_print_time));
        res.insert("extruder".into(), PyVal::Str(extruder.get_name()));
        res.insert("position".into(), coord_pyval(&commanded_pos[..4]));
        res.insert("max_velocity".into(), PyVal::Float(self.max_velocity.get()));
        res.insert("max_accel".into(), PyVal::Float(self.max_accel.get()));
        res.insert("minimum_cruise_ratio".into(), PyVal::Float(self.min_cruise_ratio.get()));
        res.insert("square_corner_velocity".into(), PyVal::Float(self.square_corner_velocity.get()));
        Some(PyVal::Dict(res))
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        let est_print_time = self.mcu.estimated_print_time(eventtime);
        let buffer_time = self.print_time.get() - est_print_time;
        let is_active = buffer_time > -60.0 || self.special_queuing_state.borrow().is_empty();
        Some((
            is_active,
            format!(
                "print_time={:.3} buffer_time={:.3} print_stall={}",
                self.print_time.get(),
                py_max(buffer_time, 0.0),
                self.print_stall.get()
            ),
        ))
    }
    fn teardown(&self) {
        self.extra_axes.borrow_mut().clear();
        *self.kin.borrow_mut() = None;
        if let Some(t) = self.priming_timer.borrow_mut().take() {
            self.reactor.unregister_timer(&t);
        }
    }
}

// Support common G-Code commands relative to the toolhead
struct ToolHeadCommandHelper {
    printer: Rc<Printer>,
    toolhead: Rc<ToolHead>,
}

const CMD_SET_VELOCITY_LIMIT_HELP: &str = "Set printer velocity limits";
const CMD_RESET_VELOCITY_LIMIT_HELP: &str = "Reset printer velocity limits";

impl ToolHeadCommandHelper {
    fn register(config: &ConfigWrapper) -> KResult<()> {
        let printer = config.get_printer();
        let toolhead = printer.lookup::<ToolHead>("toolhead")?;
        let h = Rc::new(ToolHeadCommandHelper { printer: printer.clone(), toolhead });
        // Register commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let c = h.clone();
        gcode.register_command("G4", Some(Rc::new(move |g| c.cmd_g4(g))), false, None)?;
        let c = h.clone();
        gcode.register_command("M400", Some(Rc::new(move |g| c.cmd_m400(g))), false, None)?;
        let c = h.clone();
        gcode.register_command(
            "SET_VELOCITY_LIMIT",
            Some(Rc::new(move |g| c.cmd_set_velocity_limit(g))),
            false,
            Some(CMD_SET_VELOCITY_LIMIT_HELP),
        )?;
        let c = h.clone();
        gcode.register_command(
            "RESET_VELOCITY_LIMIT",
            Some(Rc::new(move |g| c.cmd_reset_velocity_limit(g))),
            false,
            Some(CMD_RESET_VELOCITY_LIMIT_HELP),
        )?;
        let c = h;
        gcode.register_command("M204", Some(Rc::new(move |g| c.cmd_m204(g))), false, None)?;
        Ok(())
    }

    fn cmd_g4(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Dwell
        let delay = gcmd.get_float("P", Some(0.0), Lim::new().min(0.0))? / 1000.0;
        self.toolhead.dwell(delay)
    }

    fn cmd_m400(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        // Wait for current moves to finish
        self.toolhead.wait_moves()
    }

    fn cmd_set_velocity_limit(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let max_velocity = gcmd.get_float_opt("VELOCITY", Lim::new().above(0.0))?;
        let max_accel = gcmd.get_float_opt("ACCEL", Lim::new().above(0.0))?;
        let square_corner_velocity = gcmd.get_float_opt("SQUARE_CORNER_VELOCITY", Lim::new().min(0.0))?;
        let min_cruise_ratio = gcmd.get_float_opt("MINIMUM_CRUISE_RATIO", Lim::new().min(0.0).below(1.0))?;
        let kin = self.toolhead.get_kinematics();
        let (mv, ma, scv, mcr) =
            self.toolhead.set_max_velocities(max_velocity, max_accel, square_corner_velocity, min_cruise_ratio);
        let mut msg = vec![format!("max_velocity: {:.6}", mv), format!("max_accel: {:.6}", ma)];
        for (attr, param, lim) in [
            ("max_x_velocity", "X_VELOCITY", Lim::new()),
            ("max_x_accel", "X_ACCEL", Lim::new()),
            ("max_y_velocity", "Y_VELOCITY", Lim::new()),
            ("max_y_accel", "Y_ACCEL", Lim::new()),
            ("max_z_velocity", "Z_VELOCITY", Lim::new().above(0.0)),
            ("max_z_accel", "Z_ACCEL", Lim::new().above(0.0)),
        ] {
            if let Some(cell) = kin.limit(attr) {
                if let Some(v) = gcmd.get_float_opt(param, lim)? {
                    cell.set(v);
                }
                msg.push(format!("{}: {:.6}", attr, cell.get()));
            }
        }
        msg.push(format!("minimum_cruise_ratio: {:.6}", mcr));
        msg.push(format!("square_corner_velocity: {:.6}", scv));

        if get_danger_options().log_velocity_limit_changes {
            self.printer.set_rollover_info("toolhead", Some(&format!("toolhead: {}", msg.join(" "))), true);
            if max_velocity.is_none() && max_accel.is_none() && square_corner_velocity.is_none() && min_cruise_ratio.is_none()
            {
                gcmd.respond_info(&msg.join("\n"), false);
            }
        }
        Ok(())
    }

    fn cmd_reset_velocity_limit(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let kin = self.toolhead.get_kinematics();
        let orig_cfg = self.toolhead.orig_cfg.borrow().clone();
        let (mv, ma, scv, mcr) = self.toolhead.set_max_velocities(
            Some(orig_cfg["max_velocity"]),
            Some(orig_cfg["max_accel"]),
            Some(orig_cfg["square_corner_velocity"]),
            Some(orig_cfg["min_cruise_ratio"]),
        );
        let mut msg = vec![format!("max_velocity: {:.6}", mv), format!("max_accel: {:.6}", ma)];
        for attr in ["max_x_velocity", "max_x_accel", "max_y_velocity", "max_y_accel", "max_z_velocity", "max_z_accel"] {
            if let Some(cell) = kin.limit(attr) {
                cell.set(orig_cfg[attr]);
                msg.push(format!("{}: {:.6}", attr, cell.get()));
            }
        }
        msg.push(format!("minimum_cruise_ratio: {:.6}", mcr));
        msg.push(format!("square_corner_velocity: {:.6}", scv));
        if get_danger_options().log_velocity_limit_changes {
            gcmd.respond_info(&msg.join("\n"), false);
        }
        Ok(())
    }

    fn cmd_m204(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Use S for accel
        let mut accel = gcmd.get_float_opt("S", Lim::new().above(0.0))?;
        if accel.is_none() {
            // Use minimum of P and T for accel
            let p = gcmd.get_float_opt("P", Lim::new().above(0.0))?;
            let t = gcmd.get_float_opt("T", Lim::new().above(0.0))?;
            let (Some(p), Some(t)) = (p, t) else {
                gcmd.respond_info(&format!("Invalid M204 command \"{}\"", gcmd.get_commandline()), true);
                return Ok(());
            };
            accel = Some(py_min(p, t));
        }
        self.toolhead.set_max_velocities(None, accel, None, None);
        Ok(())
    }
}

pub fn add_printer_objects(printer: &Rc<Printer>, config: &ConfigWrapper) -> KResult<()> {
    printer.add_object("toolhead", ToolHead::new(config)?)?;
    ToolHeadCommandHelper::register(config)?;
    // Load default extruder objects
    kinematics_extruder::add_printer_objects(config)?;
    // Load some default modules
    let modules = ["gcode_move", "homing", "idle_timeout", "statistics", "manual_probe", "tuning_tower", "garbage_collection"];
    for module_name in modules {
        printer.load_object(config, module_name, true)?;
    }
    Ok(())
}
