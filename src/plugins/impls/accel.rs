// klippy/extras: adxl345.ADXL345, lis2dw.LIS2DW (register access, their
// batch_bulk) and bulk_sensor.BatchBulkHelper.is_started: what a plugin
// driving the chip itself uses (knock)
use super::*;
use crate::extras::adxl345::{AccelChip, ADXL345};
use crate::extras::bulk_sensor::BatchBulkHelper;
use crate::extras::lis2dw::LIS2DW;
use crate::plugins::convert::ToPy;
use crate::{method, ro_attr};

// set_reg(reg, val, minclock=0): kaliru's chips write at once (minclock 0)
fn no_minclock(minclock: i64) -> KResult<()> {
    if minclock != 0 {
        return Err(KError::Py("NotImplementedError".into(), "kaliru: set_reg with a minclock".into()));
    }
    Ok(())
}

fn batch_bulk(b: &std::cell::RefCell<Option<Rc<BatchBulkHelper>>>) -> KResult<PyRef> {
    match b.borrow().clone() {
        Some(b) => b.to_py(),
        None => Ok(py_none()),
    }
}

pub fn register(r: &mut Registry) {
    method!(r, "klippy.extras.adxl345.ADXL345", ADXL345, read_reg(c, reg: i64) => c.read_reg(reg));
    method!(r, "klippy.extras.adxl345.ADXL345", ADXL345, set_reg(c, reg: i64, val: i64, minclock: i64) => {
        no_minclock(minclock)?;
        c.set_reg(reg, val)
    });
    r.attr("klippy.extras.adxl345.ADXL345.batch_bulk", |o| batch_bulk(&objects::unwrap_as::<ADXL345>(o)?.batch_bulk), None);

    method!(r, "klippy.extras.lis2dw.LIS2DW", LIS2DW, read_reg(c, reg: i64) => c.read_reg(reg));
    method!(r, "klippy.extras.lis2dw.LIS2DW", LIS2DW, set_reg(c, reg: i64, val: i64, minclock: i64) => {
        no_minclock(minclock)?;
        c.set_reg(reg, val)
    });
    r.attr("klippy.extras.lis2dw.LIS2DW.batch_bulk", |o| batch_bulk(&objects::unwrap_as::<LIS2DW>(o)?.batch_bulk), None);
    ro_attr!(r, "klippy.extras.lis2dw.LIS2DW.lis_type", LIS2DW, |c| c.lis_type.to_string());

    ro_attr!(r, "klippy.extras.bulk_sensor.BatchBulkHelper.is_started", BatchBulkHelper, |b| b.is_started.get());
}
