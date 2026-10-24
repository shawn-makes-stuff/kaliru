// Ports of klippy/kinematics
//
// python loads the kinematics module named by [printer] kinematics with
// importlib; KINEMATICS lists the ported ones (and the known unported ones,
// which raise like a load failure).
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::pyval::PyVal;
use crate::extras::homing::Homing;
use crate::stepper::{McuStepper, PrinterRail, RailRef};
use crate::toolhead::{Move, ToolHead};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::Cell;
use std::rc::Rc;

pub mod cartesian;
pub mod delta;
pub mod deltesian;
pub mod extruder;
pub mod idex_modes;
pub mod limited;
pub mod none;
pub mod polar;
pub mod rotary_delta;
pub mod winch;

/// what toolhead.py uses of a kinematics object
pub trait Kinematics {
    /// the Rust type's path (python plugins find the Kalico class by it)
    fn type_name(&self) -> &'static str {
        std::any::type_name::<Self>()
    }
    fn get_steppers(&self) -> Vec<Rc<McuStepper>>;
    /// calc_position(stepper_positions) with positions by stepper name
    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64>;
    fn check_move(&self, mv: &mut Move) -> KResult<()>;
    fn home(&self, homing_state: &Rc<Homing>) -> KResult<()>;
    /// self.rails (python raises AttributeError for kinematics without it)
    fn rails(&self) -> Vec<RailRef> {
        vec![]
    }
    fn set_position(&self, newpos: &[f64], homing_axes: &str) -> KResult<()>;
    fn clear_homing_state(&self, clear_axes: &str);
    fn get_status(&self, eventtime: f64) -> IndexMap<String, PyVal>;
    /// self.supports_dual_carriage
    fn supports_dual_carriage(&self) -> bool {
        false
    }
    /// the kinematics' max_x_velocity / max_x_accel / ... attribute, None
    /// where python's hasattr() is false
    fn limit(&self, _name: &str) -> Option<&Cell<f64>> {
        None
    }
    /// update_limits(i, range) (cartesian style kinematics, for idex_modes)
    fn update_limits(&self, _i: usize, _range: (f64, f64)) {}
    /// home_axis(homing_state, axis, rail) (for idex_modes)
    fn home_axis(&self, _homing_state: &Rc<Homing>, _axis: usize, _rail: &Rc<PrinterRail>) -> KResult<()> {
        Ok(())
    }
    /// get_calibration() of the delta style kinematics (DELTA_CALIBRATE)
    fn get_calibration(&self) -> Option<Rc<dyn delta::Calibration>> {
        None
    }
    fn as_any(&self) -> &dyn Any;
}

pub type LoadKinematics = fn(&Rc<ToolHead>, &ConfigWrapper) -> KResult<Rc<dyn Kinematics>>;

/// klippy/kinematics/*.py that have load_kinematics(), None where not
/// ported yet
pub static KINEMATICS: &[(&str, Option<LoadKinematics>)] = &[
    ("cartesian", Some(cartesian::load_kinematics)),
    ("corexy", Some(cartesian::load_corexy)),
    ("corexz", Some(cartesian::load_corexz)),
    ("delta", Some(delta::load_kinematics)),
    ("deltesian", Some(deltesian::load_kinematics)),
    ("hybrid_corexy", Some(cartesian::load_hybrid_corexy)),
    ("hybrid_corexz", Some(cartesian::load_hybrid_corexz)),
    ("limited_cartesian", Some(limited::load_limited_cartesian)),
    ("limited_corexy", Some(limited::load_limited_corexy)),
    ("limited_corexz", Some(limited::load_limited_corexz)),
    ("none", Some(none::load_kinematics)),
    ("polar", Some(polar::load_kinematics)),
    ("rotary_delta", Some(rotary_delta::load_kinematics)),
    ("winch", Some(winch::load_kinematics)),
];
