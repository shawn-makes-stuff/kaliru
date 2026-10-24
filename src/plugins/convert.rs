// Conversions between kaliru values and python objects for the handlers
// (impls/): ToPy for what a kaliru method returns, FromPy for its arguments.
// Kaliru objects go through the object table (the same python object each
// time), printer objects through pobj_to_py / py_to_pobj.
use super::capi::*;
use super::objects;
use super::py::*;
use crate::error::{KError, KResult};
use crate::printer::PObj;
use crate::pyval::PyVal;
use std::rc::Rc;

pub trait ToPy {
    fn to_py(self) -> KResult<PyRef>;
}

pub trait FromPy: Sized {
    fn from_py(o: &PyRef) -> KResult<Self>;
}

impl ToPy for PyRef {
    fn to_py(self) -> KResult<PyRef> {
        Ok(self)
    }
}
impl ToPy for () {
    fn to_py(self) -> KResult<PyRef> {
        Ok(py_none())
    }
}
impl ToPy for f64 {
    fn to_py(self) -> KResult<PyRef> {
        Ok(py_float(self))
    }
}
impl ToPy for bool {
    fn to_py(self) -> KResult<PyRef> {
        Ok(py_bool(self))
    }
}
macro_rules! int_to_py {
    ($($t:ty),*) => { $( impl ToPy for $t { fn to_py(self) -> KResult<PyRef> { Ok(py_int(self as i64)) } } )* };
}
int_to_py!(i64, i32, u32, u64, usize, u8, i8, u16, i16);
impl ToPy for String {
    fn to_py(self) -> KResult<PyRef> {
        Ok(py_str(&self))
    }
}
impl ToPy for &str {
    fn to_py(self) -> KResult<PyRef> {
        Ok(py_str(self))
    }
}
impl ToPy for &String {
    fn to_py(self) -> KResult<PyRef> {
        Ok(py_str(self))
    }
}
impl ToPy for PyVal {
    fn to_py(self) -> KResult<PyRef> {
        Ok(from_pyval(&self))
    }
}
impl ToPy for &PyVal {
    fn to_py(self) -> KResult<PyRef> {
        Ok(from_pyval(self))
    }
}
impl<T: ToPy> ToPy for Option<T> {
    fn to_py(self) -> KResult<PyRef> {
        match self {
            Some(v) => v.to_py(),
            None => Ok(py_none()),
        }
    }
}
impl<T: ToPy> ToPy for KResult<T> {
    fn to_py(self) -> KResult<PyRef> {
        self?.to_py()
    }
}
/// a python list
impl<T: ToPy> ToPy for Vec<T> {
    fn to_py(self) -> KResult<PyRef> {
        Ok(py_list(self.into_iter().map(|v| v.to_py()).collect::<KResult<Vec<_>>>()?))
    }
}
impl<A: ToPy, B: ToPy> ToPy for (A, B) {
    fn to_py(self) -> KResult<PyRef> {
        Ok(py_tuple(vec![self.0.to_py()?, self.1.to_py()?]))
    }
}
impl<A: ToPy, B: ToPy, C: ToPy> ToPy for (A, B, C) {
    fn to_py(self) -> KResult<PyRef> {
        Ok(py_tuple(vec![self.0.to_py()?, self.1.to_py()?, self.2.to_py()?]))
    }
}
/// a kaliru object: its python object of the mirrored class
impl<T: 'static> ToPy for Rc<T> {
    fn to_py(self) -> KResult<PyRef> {
        objects::wrap(self)
    }
}
impl ToPy for PObj {
    fn to_py(self) -> KResult<PyRef> {
        super::pobj_to_py(&self)
    }
}

impl FromPy for PyRef {
    fn from_py(o: &PyRef) -> KResult<Self> {
        Ok(o.clone())
    }
}
impl FromPy for f64 {
    fn from_py(o: &PyRef) -> KResult<Self> {
        o.as_f64()
    }
}
/// python truthiness (Kalico passes 0/1 and None as flags too)
impl FromPy for bool {
    fn from_py(o: &PyRef) -> KResult<Self> {
        o.is_true()
    }
}
macro_rules! int_from_py {
    ($($t:ty),*) => { $( impl FromPy for $t { fn from_py(o: &PyRef) -> KResult<Self> { Ok(o.as_i64()? as $t) } } )* };
}
int_from_py!(i64, i32, u32, u64, usize, u8, i8, u16, i16);
impl FromPy for String {
    fn from_py(o: &PyRef) -> KResult<Self> {
        o.as_string()
    }
}
impl FromPy for PyVal {
    fn from_py(o: &PyRef) -> KResult<Self> {
        to_pyval(o)
    }
}
impl<T: FromPy> FromPy for Option<T> {
    fn from_py(o: &PyRef) -> KResult<Self> {
        if o.is_none() { Ok(None) } else { T::from_py(o).map(Some) }
    }
}
/// any iterable (list, tuple, ...)
impl<T: FromPy> FromPy for Vec<T> {
    fn from_py(o: &PyRef) -> KResult<Self> {
        let list = lookup("builtins", "list")?.call(&[o])?;
        let n = unsafe { PyList_Size(list.as_ptr()) };
        (0..n).map(|i| T::from_py(&unsafe { PyRef::borrowed(PyList_GetItem(list.as_ptr(), i)) })).collect()
    }
}
impl<A: FromPy, B: FromPy> FromPy for (A, B) {
    fn from_py(o: &PyRef) -> KResult<Self> {
        let v: Vec<PyRef> = Vec::from_py(o)?;
        if v.len() != 2 {
            return Err(KError::Py("ValueError".into(), format!("expected 2 values, got {}", v.len())));
        }
        Ok((A::from_py(&v[0])?, B::from_py(&v[1])?))
    }
}
/// the kaliru object behind a python object of a mirrored class
impl<T: 'static> FromPy for Rc<T> {
    fn from_py(o: &PyRef) -> KResult<Self> {
        objects::unwrap_as::<T>(o)
    }
}
impl FromPy for PObj {
    fn from_py(o: &PyRef) -> KResult<Self> {
        Ok(super::py_to_pobj(o))
    }
}

/// Handler one-liners:
///   method!(r, "klippy.toolhead.ToolHead", ToolHead, get_position(t) => t.get_position());
///   method!(r, "...", ToolHead, set_position(t, pos: Vec<f64>, axes: String) => t.set_position(&pos, &axes));
///   function!(r, "klippy.extras.force_move", calc_move_time(dist: f64, speed: f64, accel: f64) => ...);
#[macro_export]
macro_rules! method {
    ($r:expr, $class:literal, $ty:ty, $name:ident($s:ident $(, $arg:ident: $at:ty)*) => $body:expr) => {
        $r.method(concat!($class, ".", stringify!($name)), |a: &$crate::plugins::native::Args| {
            #[allow(unused_mut, unused_variables)]
            let mut args = a.args.iter();
            let $s = a.this::<$ty>()?;
            $( let $arg: $at = $crate::plugins::convert::FromPy::from_py(args.next().expect("mirrored signature"))?; )*
            $crate::plugins::convert::ToPy::to_py($body)
        })
    };
}

#[macro_export]
macro_rules! function {
    ($r:expr, $module:literal, $name:ident($($arg:ident: $at:ty),*) => $body:expr) => {
        $r.method(concat!($module, ".", stringify!($name)), |a: &$crate::plugins::native::Args| {
            #[allow(unused_mut, unused_variables)]
            let mut args = a.args.iter();
            $( let $arg: $at = $crate::plugins::convert::FromPy::from_py(args.next().expect("mirrored signature"))?; )*
            $crate::plugins::convert::ToPy::to_py($body)
        })
    };
}

/// A Cell field as an attribute python reads and writes:
///   cell_attr!(r, "klippy.extras.idle_timeout.IdleTimeout.idle_timeout", IdleTimeout, idle_timeout);
#[macro_export]
macro_rules! cell_attr {
    ($r:expr, $qual:literal, $ty:ty, $field:ident) => {
        $r.attr(
            $qual,
            |o| $crate::plugins::convert::ToPy::to_py($crate::plugins::objects::unwrap_as::<$ty>(o)?.$field.get()),
            Some(|o, v| {
                $crate::plugins::objects::unwrap_as::<$ty>(o)?.$field.set($crate::plugins::convert::FromPy::from_py(v)?);
                Ok(())
            }),
        )
    };
}

/// A read-only attribute computed from the object:
///   ro_attr!(r, "klippy.extras.idle_timeout.IdleTimeout.state", IdleTimeout, |s| s.state.borrow().to_string());
#[macro_export]
macro_rules! ro_attr {
    ($r:expr, $qual:literal, $ty:ty, |$s:ident| $body:expr) => {
        $r.attr(
            $qual,
            |o| {
                let $s = $crate::plugins::objects::unwrap_as::<$ty>(o)?;
                let v = $body;
                $crate::plugins::convert::ToPy::to_py(v)
            },
            None,
        )
    };
}

/// A RefCell<Vec<_>> field as a list attribute (prelude.py ListView: item
/// assignment writes through, as on python's list)
#[macro_export]
macro_rules! vec_attr {
    ($r:expr, $qual:literal, $ty:ty, $field:ident) => {
        $r.list(
            $qual,
            $crate::plugins::native::ListField {
                get: |o, _f| {
                    let s = $crate::plugins::objects::unwrap_as::<$ty>(o)?;
                    let items = s.$field.borrow().clone();
                    items.into_iter().map($crate::plugins::convert::ToPy::to_py).collect()
                },
                set: |o, _f, items| {
                    let s = $crate::plugins::objects::unwrap_as::<$ty>(o)?;
                    let v = items.iter().map($crate::plugins::convert::FromPy::from_py).collect::<$crate::error::KResult<Vec<_>>>()?;
                    *s.$field.borrow_mut() = v;
                    Ok(())
                },
            },
        )
    };
}
