// Automatic calibration of input shapers
// (port of klippy/extras/shaper_calibrate.py)
//
// Copyright (C) 2020-2024  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The numpy computations follow numpy 2.2's own evaluation order (pairwise
// sums, pocketfft's rfft in numpy_fft.rs); checked against python with
// tools/gen_shaper_cases.py.
use crate::error::{KError, KResult};
use crate::extras::adxl345::AccelMeasurement;
use crate::extras::shaper_defs::{Shaper, DEFAULT_DAMPING_RATIO, INPUT_SHAPERS, SHAPER_VIBRATION_REDUCTION};
use crate::gcode::GCodeDispatch;
use crate::mathutil::{np_interp, np_kaiser, np_pairwise_sum};
use crate::printer::Printer;
use crate::pyfmt::{py_max, py_min, py_sum};
use crate::pyformat::percent_format;
use crate::pyval::PyVal;
use std::f64::consts::PI;
use std::rc::Rc;

const MIN_FREQ: f64 = 5.0;
const MAX_FREQ: f64 = 200.0;
const WINDOW_T_SEC: f64 = 0.5;
const MAX_SHAPER_FREQ: f64 = 150.0;

const TEST_DAMPING_RATIOS: [f64; 3] = [0.075, 0.1, 0.15];

pub const AUTOTUNE_SHAPERS: [&str; 5] = ["zv", "mzv", "ei", "2hump_ei", "3hump_ei"];

//######################################################################
// Frequency response calculation and shaper auto-tuning
//######################################################################

/// np.maximum of two values (NaN propagates)
fn np_maximum(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else if a >= b {
        a
    } else {
        b
    }
}

#[derive(Clone, Debug)]
pub struct CalibrationData {
    pub freq_bins: Vec<f64>,
    pub psd_sum: Vec<f64>,
    pub psd_x: Vec<f64>,
    pub psd_y: Vec<f64>,
    pub psd_z: Vec<f64>,
    data_sets: i64,
}

impl CalibrationData {
    fn new(freq_bins: Vec<f64>, psd_sum: Vec<f64>, psd_x: Vec<f64>, psd_y: Vec<f64>, psd_z: Vec<f64>) -> Self {
        CalibrationData { freq_bins, psd_sum, psd_x, psd_y, psd_z, data_sets: 1 }
    }

    fn psd_list(&mut self) -> [&mut Vec<f64>; 4] {
        [&mut self.psd_sum, &mut self.psd_x, &mut self.psd_y, &mut self.psd_z]
    }

    pub fn add_data(&mut self, other: &CalibrationData) {
        let joined_data_sets = self.data_sets + other.data_sets;
        let freq_bins = self.freq_bins.clone();
        let data_sets = self.data_sets as f64;
        let other_psds = [&other.psd_sum, &other.psd_x, &other.psd_y, &other.psd_z];
        for (psd, other_psd) in self.psd_list().into_iter().zip(other_psds) {
            // `other` data may be defined at different frequency bins,
            // interpolating to fix that.
            for (i, v) in psd.iter_mut().enumerate() {
                let other_normalized = other.data_sets as f64 * np_interp(freq_bins[i], &other.freq_bins, other_psd);
                *v *= data_sets;
                *v = (*v + other_normalized) * (1.0 / joined_data_sets as f64);
            }
        }
        self.data_sets = joined_data_sets;
    }

    pub fn normalize_to_frequencies(&mut self) {
        let freq_bins = self.freq_bins.clone();
        for psd in self.psd_list() {
            for (v, f) in psd.iter_mut().zip(&freq_bins) {
                // Avoid division by zero errors
                *v /= f + 0.1;
                // Remove low-frequency noise
                if *f < 2.0 * MIN_FREQ {
                    let r = 2.0 * MIN_FREQ / (f + 0.1);
                    *v *= (-(r * r) + 1.0).exp();
                }
            }
        }
    }
}

/// a fitted shaper (CalibrationResult)
#[derive(Clone, Debug)]
pub struct CalibrationResult {
    pub name: &'static str,
    pub freq: f64,
    pub vals: Vec<f64>,
    pub vibrs: f64,
    pub smoothing: f64,
    pub score: f64,
    pub max_accel: f64,
}

/// mean of each column of a (rows x cols) column-major block:
/// np.mean(x, axis=0) of the strided window view (pairwise per column)
fn column_means(cols: &[Vec<f64>]) -> Vec<f64> {
    cols.iter().map(|c| np_pairwise_sum(c) / c.len() as f64).collect()
}

/// _psd(x, fs, nfft): Welch's PSD
fn psd(x: &[f64], fs: f64, nfft: usize) -> (Vec<f64>, Vec<f64>) {
    // Calculate power spectral density (PSD) using Welch's algorithm
    let window = np_kaiser(nfft, 6.0);
    // Compensation for windowing loss
    let w2: Vec<f64> = window.iter().map(|w| w * w).collect();
    let scale = 1.0 / np_pairwise_sum(&w2);

    // Split into overlapping windows of size nfft
    let overlap = nfft / 2;
    let step_between_windows = nfft - overlap;
    let n_windows = (x.len() - overlap) / step_between_windows;
    let windows: Vec<Vec<f64>> = (0..n_windows).map(|w| x[w * step_between_windows..w * step_between_windows + nfft].to_vec()).collect();

    // First detrend, then apply windowing function
    let means = column_means(&windows);
    let rfft = crate::numpy_fft::rfft_plan(nfft);
    let nout = nfft / 2 + 1;
    let k = scale / fs;
    let mut result = vec![vec![0.0; n_windows]; nout];
    for (w, col) in windows.iter().enumerate() {
        let xw: Vec<f64> = col.iter().zip(&window).map(|(v, win)| win * (v - means[w])).collect();
        // Calculate frequency response for each window using FFT
        let r = rfft(&xw);
        for (i, (re, im)) in r.iter().enumerate() {
            // np.conjugate(result) * result: numpy's AVX2 complex multiply
            // is an fmaddsub, real = fma(ar, br, -(ai * bi))
            let mut p = re.mul_add(*re, -((-im) * im));
            p *= k;
            // For one-sided FFT output the response must be doubled, except
            // the last point for unpaired Nyquist frequency (assuming even nfft)
            // and the 'DC' term (0 Hz)
            if i > 0 && i < nout - 1 {
                p *= 2.0;
            }
            result[i][w] = p;
        }
    }

    // Welch's algorithm: average response over windows (numpy keeps the
    // contiguous frequency axis innermost: a sequential sum per bin)
    let psd: Vec<f64> = result.iter().map(|row| row[1..].iter().fold(row[0], |a, b| a + b) / row.len() as f64).collect();

    // Calculate the frequency bins: np.fft.rfftfreq(nfft, 1.0 / fs)
    let val = 1.0 / (nfft as f64 * (1.0 / fs));
    let freqs = (0..nout).map(|i| i as f64 * val).collect();
    (freqs, psd)
}

/// calc_freq_response(raw_values)
pub fn calc_freq_response(samples: &[AccelMeasurement]) -> Option<CalibrationData> {
    if samples.is_empty() {
        return None;
    }
    let n = samples.len();
    let t = samples[n - 1].time - samples[0].time;
    let sampling_freq = n as f64 / t;
    // Round up to the nearest power of 2 for faster FFT
    let v = (sampling_freq * WINDOW_T_SEC - 1.0) as i64;
    let bit_length = 64 - v.unsigned_abs().leading_zeros();
    let m = 1usize << bit_length;
    if n <= m {
        return None;
    }

    // Calculate PSD (power spectral density) of vibrations per
    // frequency bins (the same bins for X, Y, and Z)
    let col = |f: fn(&AccelMeasurement) -> f64| samples.iter().map(f).collect::<Vec<f64>>();
    let (fx, px) = psd(&col(|s| s.accel_x), sampling_freq, m);
    let (_fy, py) = psd(&col(|s| s.accel_y), sampling_freq, m);
    let (_fz, pz) = psd(&col(|s| s.accel_z), sampling_freq, m);
    let sum: Vec<f64> = px.iter().zip(&py).zip(&pz).map(|((a, b), c)| a + b + c).collect();
    Some(CalibrationData::new(fx, sum, px, py, pz))
}

fn estimate_shaper(shaper: &Shaper, test_damping_ratio: f64, test_freqs: &[f64]) -> Vec<f64> {
    let (a, t) = shaper;
    let inv_d = 1.0 / a.iter().fold(0.0, |s, v| s + v);
    let t_last = t[t.len() - 1];
    let df = (1.0 - test_damping_ratio.powf(2.0)).sqrt();
    test_freqs
        .iter()
        .map(|f| {
            let omega = 2.0 * PI * f;
            let damping = test_damping_ratio * omega;
            let omega_d = omega * df;
            let (mut s, mut c) = (0.0, 0.0);
            for (ai, ti) in a.iter().zip(t) {
                let w = ai * ((-damping) * (t_last - ti)).exp();
                s += w * (omega_d * ti).sin();
                c += w * (omega_d * ti).cos();
            }
            (s * s + c * c).sqrt() * inv_d
        })
        .collect()
}

/// (remaining vibrations ratio, vals)
fn estimate_remaining_vibrations(shaper: &Shaper, test_damping_ratio: f64, freq_bins: &[f64], psd: &[f64]) -> (f64, Vec<f64>) {
    let vals = estimate_shaper(shaper, test_damping_ratio, freq_bins);
    // The input shaper can only reduce the amplitude of vibrations by
    // SHAPER_VIBRATION_REDUCTION times, so all vibrations below that
    // threshold can be igonred
    let psd_max = psd.iter().copied().fold(f64::NEG_INFINITY, np_maximum);
    let vibr_threshold = psd_max / SHAPER_VIBRATION_REDUCTION;
    let rem: Vec<f64> = vals.iter().zip(psd).map(|(v, p)| np_maximum(v * p - vibr_threshold, 0.0)).collect();
    let all: Vec<f64> = psd.iter().map(|p| np_maximum(p - vibr_threshold, 0.0)).collect();
    (np_pairwise_sum(&rem) / np_pairwise_sum(&all), vals)
}

fn get_shaper_smoothing(shaper: &Shaper, accel: f64, scv: f64) -> f64 {
    let half_accel = accel * 0.5;

    let (a, t) = shaper;
    let inv_d = 1.0 / py_sum(a.iter().copied());
    let n = t.len();
    // Calculate input shaper shift
    let ts = py_sum((0..n).map(|i| a[i] * t[i])) * inv_d;

    // Calculate offset for 90 and 180 degrees turn
    let (mut offset_90, mut offset_180) = (0.0, 0.0);
    for i in 0..n {
        if t[i] >= ts {
            // Calculate offset for one of the axes
            offset_90 += a[i] * (scv + half_accel * (t[i] - ts)) * (t[i] - ts);
        }
        offset_180 += a[i] * half_accel * (t[i] - ts).powf(2.0);
    }
    offset_90 *= inv_d * 2f64.sqrt();
    offset_180 *= inv_d;
    py_max(offset_90, offset_180)
}

fn bisect(func: impl Fn(f64) -> bool) -> f64 {
    let (mut left, mut right) = (1.0, 1.0);
    if !func(1e-9) {
        return 0.0;
    }
    while !func(left) {
        right = left;
        left *= 0.5;
    }
    if right == left {
        while func(right) {
            right *= 2.0;
        }
    }
    while right - left > 1e-8 {
        let middle = (left + right) * 0.5;
        if func(middle) {
            left = middle;
        } else {
            right = middle;
        }
    }
    left
}

pub fn find_shaper_max_accel(shaper: &Shaper, scv: f64) -> f64 {
    // Just some empirically chosen value which produces good projections
    // for max_accel without much smoothing
    const TARGET_SMOOTHING: f64 = 0.12;
    bisect(|test_accel| get_shaper_smoothing(shaper, test_accel, scv) <= TARGET_SMOOTHING)
}

/// np.arange(start, stop, step) of floats
fn np_arange(start: f64, stop: f64, step: f64) -> Vec<f64> {
    let len = ((stop - start) / step).ceil();
    if len.is_nan() || len <= 0.0 {
        return vec![];
    }
    let len = len as usize;
    let second = start + step;
    let delta = second - start;
    (0..len)
        .map(|i| match i {
            0 => start,
            1 => second,
            _ => start + i as f64 * delta,
        })
        .collect()
}

/// fit_shaper(shaper_cfg, calibration_data, shaper_freqs=None, damping_ratio,
/// scv, max_smoothing, test_damping_ratios=None, max_freq)
pub fn fit_shaper(
    shaper_index: usize,
    calibration_data: &CalibrationData,
    damping_ratio: Option<f64>,
    scv: f64,
    max_smoothing: Option<f64>,
    max_freq: Option<f64>,
) -> CalibrationResult {
    let shaper_cfg = &INPUT_SHAPERS[shaper_index];
    let damping_ratio = damping_ratio.filter(|d| *d != 0.0).unwrap_or(DEFAULT_DAMPING_RATIO);
    let test_damping_ratios = TEST_DAMPING_RATIOS;

    let freq_end = MAX_SHAPER_FREQ;
    let freq_start = py_min(shaper_cfg.min_freq, freq_end - 1e-7);
    let freq_step = 0.2;
    let test_freqs = np_arange(freq_start, freq_end, freq_step);

    let test_max = test_freqs.iter().copied().fold(f64::NEG_INFINITY, np_maximum);
    let max_freq = py_max(max_freq.filter(|m| *m != 0.0).unwrap_or(MAX_FREQ), test_max);

    let keep: Vec<usize> = (0..calibration_data.freq_bins.len()).filter(|i| calibration_data.freq_bins[*i] <= max_freq).collect();
    let psd: Vec<f64> = keep.iter().map(|i| calibration_data.psd_sum[*i]).collect();
    let freq_bins: Vec<f64> = keep.iter().map(|i| calibration_data.freq_bins[*i]).collect();

    let mut best_res: Option<CalibrationResult> = None;
    let mut results: Vec<CalibrationResult> = vec![];
    for &test_freq in test_freqs.iter().rev() {
        let mut shaper_vibrations = 0.0;
        let mut shaper_vals = vec![0.0; freq_bins.len()];
        let shaper = (shaper_cfg.init_func)(test_freq, damping_ratio);
        let shaper_smoothing = get_shaper_smoothing(&shaper, 5000.0, scv);
        if let (Some(ms), Some(best)) = (max_smoothing.filter(|m| *m != 0.0), &best_res) {
            if shaper_smoothing > ms {
                return best.clone();
            }
        }
        // Exact damping ratio of the printer is unknown, pessimizing
        // remaining vibrations over possible damping values
        for dr in test_damping_ratios {
            let (vibrations, vals) = estimate_remaining_vibrations(&shaper, dr, &freq_bins, &psd);
            for (s, v) in shaper_vals.iter_mut().zip(&vals) {
                *s = np_maximum(*s, *v);
            }
            if vibrations > shaper_vibrations {
                shaper_vibrations = vibrations;
            }
        }
        let max_accel = find_shaper_max_accel(&shaper, scv);
        // The score trying to minimize vibrations, but also accounting
        // the growth of smoothing. The formula itself does not have any
        // special meaning, it simply shows good results on real user data
        let shaper_score = shaper_smoothing * (shaper_vibrations.powf(1.5) + shaper_vibrations * 0.2 + 0.01);
        results.push(CalibrationResult {
            name: shaper_cfg.name,
            freq: test_freq,
            vals: shaper_vals,
            vibrs: shaper_vibrations,
            smoothing: shaper_smoothing,
            score: shaper_score,
            max_accel,
        });
        let last = results.last().unwrap();
        if best_res.as_ref().map(|b| b.vibrs > last.vibrs).unwrap_or(true) {
            // The current frequency is better for the shaper.
            best_res = Some(last.clone());
        }
    }
    // Try to find an 'optimal' shapper configuration: the one that is not
    // much worse than the 'best' one, but gives much less smoothing
    let best_res = best_res.expect("shaper frequencies");
    let mut selected = best_res.clone();
    for res in results.iter().rev() {
        if res.vibrs < best_res.vibrs * 1.1 && res.score < selected.score {
            selected = res.clone();
        }
    }
    selected
}

/// ShaperCalibrate(printer)
pub struct ShaperCalibrate {
    printer: Rc<Printer>,
}

impl ShaperCalibrate {
    pub fn new(printer: &Rc<Printer>) -> Self {
        ShaperCalibrate { printer: printer.clone() }
    }

    /// background_process_exec(method, args): python runs the method in a
    /// child process while the reactor keeps running; this runs it on a
    /// thread and polls it the same way
    fn background_process_exec<T: Send + 'static>(&self, method: impl FnOnce() -> T + Send + 'static) -> KResult<T> {
        let handle = std::thread::spawn(method);
        // Wait for the process to finish
        let reactor = self.printer.get_reactor();
        let gcode = self.printer.lookup::<GCodeDispatch>("gcode")?;
        let mut eventtime = reactor.monotonic();
        let mut last_report_time = eventtime;
        while !handle.is_finished() {
            if eventtime > last_report_time + 5.0 {
                last_report_time = eventtime;
                gcode.respond_info("Wait for calculations..", false);
            }
            eventtime = reactor.pause(eventtime + 0.1)?;
        }
        // Return results
        handle.join().map_err(|_| KError::command("Error in remote calculation: thread panicked"))
    }

    /// process_accelerometer_data(aclient) of its samples
    pub fn process_accelerometer_data(&self, samples: Vec<AccelMeasurement>, data_repr: &str) -> KResult<CalibrationData> {
        let calibration_data = self.background_process_exec(move || calc_freq_response(&samples))?;
        calibration_data.ok_or_else(|| KError::command(format!("Internal error processing accelerometer data {}", data_repr)))
    }

    /// find_best_shaper(calibration_data, shapers, damping_ratio=None,
    /// scv, shaper_freqs=None, max_smoothing, test_damping_ratios=None,
    /// max_freq, logger)
    pub fn find_best_shaper(
        &self,
        calibration_data: &CalibrationData,
        shapers: Option<&[String]>,
        scv: f64,
        max_smoothing: Option<f64>,
        max_freq: Option<f64>,
        logger: Option<&dyn Fn(&str)>,
    ) -> KResult<(CalibrationResult, Vec<CalibrationResult>)> {
        let mut best_shaper: Option<CalibrationResult> = None;
        let mut all_shapers = vec![];
        for (idx, shaper_cfg) in INPUT_SHAPERS.iter().enumerate() {
            // shapers or AUTOTUNE_SHAPERS (an empty list is falsy)
            let wanted = match shapers.filter(|s| !s.is_empty()) {
                Some(s) => s.iter().any(|n| n == shaper_cfg.name),
                None => AUTOTUNE_SHAPERS.contains(&shaper_cfg.name),
            };
            if !wanted {
                continue;
            }
            let data = calibration_data.clone();
            let shaper = self.background_process_exec(move || fit_shaper(idx, &data, None, scv, max_smoothing, max_freq))?;
            if let Some(logger) = logger {
                logger(&format!(
                    "Fitted shaper '{}' frequency = {:.1} Hz (vibrations = {:.1}%, smoothing ~= {:.3})",
                    shaper.name,
                    shaper.freq,
                    shaper.vibrs * 100.0,
                    shaper.smoothing
                ));
                logger(&format!(
                    "To avoid too much smoothing with '{}', suggested max_accel <= {:.0} mm/sec^2",
                    shaper.name,
                    crate::pyfmt::py_round(shaper.max_accel / 100.0, 0) * 100.0
                ));
            }
            let replace = match &best_shaper {
                None => true,
                Some(b) => shaper.score * 1.2 < b.score || (shaper.score * 1.05 < b.score && shaper.smoothing * 1.1 < b.smoothing),
            };
            all_shapers.push(shaper.clone());
            if replace {
                // Either the shaper significantly improves the score (by 20%),
                // or it improves the score and smoothing (by 5% and 10% resp.)
                best_shaper = Some(shaper);
            }
        }
        let best = best_shaper.ok_or_else(|| KError::Py("AttributeError".into(), "'NoneType' object has no attribute 'name'".into()))?;
        Ok((best, all_shapers))
    }

    pub fn save_calibration_data(
        &self,
        output: &str,
        calibration_data: &CalibrationData,
        shapers: Option<&[CalibrationResult]>,
        max_freq: Option<f64>,
        accel_per_hz: f64,
    ) -> KResult<()> {
        let text = format_calibration_data(calibration_data, shapers, max_freq, accel_per_hz);
        std::fs::write(output, text).map_err(|e| {
            let msg = PyVal::Tuple(vec![
                PyVal::Str("Error writing to file '%s': %s".into()),
                PyVal::Str(output.into()),
                PyVal::Str(crate::pyfmt::py_oserror(&e, output)),
            ]);
            KError::command(msg.repr())
        })
    }
}

/// the csv text save_calibration_data writes
pub fn format_calibration_data(calibration_data: &CalibrationData, shapers: Option<&[CalibrationResult]>, max_freq: Option<f64>, accel_per_hz: f64) -> String {
    let max_freq = max_freq.filter(|m| *m != 0.0).unwrap_or(MAX_FREQ);
    let pf = |fmt: &str, v: f64| percent_format(fmt, &PyVal::Float(v)).unwrap_or_default();
    let mut out = String::from("freq,psd_x,psd_y,psd_z,psd_xyz,accel_per_hz");
    let shapers = shapers.filter(|s| !s.is_empty());
    if let Some(shapers) = shapers {
        for shaper in shapers {
            out += &format!(",{}({})", shaper.name, pf("%.1f", shaper.freq));
        }
    }
    out.push('\n');
    for i in 0..calibration_data.freq_bins.len() {
        if calibration_data.freq_bins[i] >= max_freq {
            break;
        }
        out += &format!(
            "{},{},{},{},{},{}",
            pf("%.1f", calibration_data.freq_bins[i]),
            pf("%.3e", calibration_data.psd_x[i]),
            pf("%.3e", calibration_data.psd_y[i]),
            pf("%.3e", calibration_data.psd_z[i]),
            pf("%.3e", calibration_data.psd_sum[i]),
            pf("%.1f", accel_per_hz)
        );
        if let Some(shapers) = shapers {
            for shaper in shapers {
                out += &format!(",{}", pf("%.3f", shaper.vals[i]));
            }
        }
        out.push('\n');
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mathutil::numpy_tests::parse_hex;

    fn rows(v: &serde_json::Value) -> Vec<AccelMeasurement> {
        v.as_array()
            .unwrap()
            .iter()
            .map(|r| {
                let r: Vec<f64> = r.as_array().unwrap().iter().map(|x| parse_hex(x.as_str().unwrap())).collect();
                AccelMeasurement { time: r[0], accel_x: r[1], accel_y: r[2], accel_z: r[3] }
            })
            .collect()
    }

    fn hexes(v: &serde_json::Value) -> Vec<f64> {
        v.as_array().unwrap().iter().map(|x| parse_hex(x.as_str().unwrap())).collect()
    }

    fn same(a: &[f64], b: &[f64]) -> usize {
        a.iter().zip(b).filter(|(x, y)| x.to_bits() != y.to_bits()).count() + a.len().abs_diff(b.len())
    }

    /// RK_SHAPER_CASES: tools/gen_shaper_cases.py output
    #[test]
    fn shaper_vs_python() {
        let Ok(path) = std::env::var("RK_SHAPER_CASES") else { return };
        let text = std::fs::read_to_string(path).unwrap();
        let mut bad = 0;
        for (n, line) in text.lines().enumerate() {
            let rec: serde_json::Value = serde_json::from_str(line).unwrap();
            let mut d = calc_freq_response(&rows(&rec["s1"])).unwrap();
            let px = same(&d.psd_x, &hexes(&rec["psd_x"]));
            let fq = same(&d.freq_bins, &hexes(&rec["freqs"]));
            let noise = format!("{:.6}", np_pairwise_sum(&d.psd_x) / d.psd_x.len() as f64);
            if !rec["s2"].is_null() {
                d.add_data(&calc_freq_response(&rows(&rec["s2"])).unwrap());
            }
            d.normalize_to_frequencies();
            let ps = same(&d.psd_sum, &hexes(&rec["psd_sum"]));
            let opt = |k: &str| rec[k].as_f64();
            let mut logs = vec![];
            let (best, all, csv) = {
                let mut fits = vec![];
                for (idx, cfg) in INPUT_SHAPERS.iter().enumerate() {
                    if AUTOTUNE_SHAPERS.contains(&cfg.name) {
                        fits.push(fit_shaper(idx, &d, None, rec["scv"].as_f64().unwrap(), opt("max_smoothing"), opt("max_freq")));
                    }
                }
                let mut best: Option<CalibrationResult> = None;
                for s in &fits {
                    logs.push(format!("Fitted shaper '{}' frequency = {:.1} Hz (vibrations = {:.1}%, smoothing ~= {:.3})", s.name, s.freq, s.vibrs * 100.0, s.smoothing));
                    logs.push(format!("To avoid too much smoothing with '{}', suggested max_accel <= {:.0} mm/sec^2", s.name, crate::pyfmt::py_round(s.max_accel / 100.0, 0) * 100.0));
                    let replace = match &best {
                        None => true,
                        Some(b) => s.score * 1.2 < b.score || (s.score * 1.05 < b.score && s.smoothing * 1.1 < b.smoothing),
                    };
                    if replace {
                        best = Some(s.clone());
                    }
                }
                let csv = format_calibration_data(&d, Some(&fits), opt("max_freq"), 75.0);
                (best.unwrap(), fits, csv)
            };
            let _ = all;
            let want_logs: Vec<String> = rec["logs"].as_array().unwrap().iter().map(|l| l.as_str().unwrap().to_string()).collect();
            let best_ok = best.name == rec["best"][0].as_str().unwrap() && best.freq.to_bits() == parse_hex(rec["best"][1].as_str().unwrap()).to_bits();
            let csv_ok = csv == rec["csv"].as_str().unwrap();
            let ok = px == 0 && fq == 0 && ps == 0 && logs == want_logs && best_ok && csv_ok && noise == rec["noise"].as_str().unwrap();
            eprintln!("case {}: psd_x {} freqs {} psd_sum {} diffs, logs {}, best {}, csv {}, noise {}", n, px, fq, ps, logs == want_logs, best_ok, csv_ok, noise == rec["noise"].as_str().unwrap());
            if !ok {
                bad += 1;
                if logs != want_logs {
                    eprintln!("  got  {:?}\n  want {:?}", logs, want_logs);
                }
            }
        }
        assert_eq!(bad, 0);
    }
}
