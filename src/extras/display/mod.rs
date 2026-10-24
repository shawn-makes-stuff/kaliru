// Basic LCD display support (port of klippy/extras/display/__init__.py and
// display.py; the display_template / display_data / display_glyph caches
// are extras/display_templates.rs)
//
// Copyright (C) 2018-2022  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2018  Aleph Objects, Inc <marcio@alephobjects.com>
// Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
pub mod aip31068_spi;
pub mod font8x14;
pub mod hd44780;
pub mod hd44780_spi;
pub mod menu;
pub mod menu_keys;
pub mod st7920;
pub mod uc1701;

use crate::configfile::{ConfigWrapper, Dflt};
use crate::error::{KError, KResult};
use crate::extras::display_templates::{lookup_display_templates, Glyph, PrinterDisplayTemplate};
use crate::extras::gcode_macro::{create_template_context, kerror_exc};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::jinja::value::{CallCtx, Exc, Object, VResult, Value};
use crate::logging;
use crate::mcu::Mcu;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::reactor::{Reactor, Timer, NOW};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

// Normal time between each screen redraw
const REDRAW_TIME: f64 = 0.500;
// Minimum time between screen redraws
const REDRAW_MIN_TIME: f64 = 0.100;

/// the low-level lcd handler interface (python duck types the chips)
pub trait LcdChip {
    fn init(&self) -> KResult<()>;
    fn flush(&self) -> KResult<()>;
    fn clear(&self);
    fn write_text(&self, x: i64, y: i64, data: &[u8]);
    fn write_graphics(&self, x: i64, y: i64, data: &[u8]);
    /// the width the glyph took (0 when unknown)
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> i64;
    fn set_glyphs(&self, glyphs: &IndexMap<String, Glyph>);
    fn get_dimensions(&self) -> (i64, i64);
    /// self.mcu
    fn get_mcu(&self) -> Rc<Mcu>;
}

type ChipFactory = fn(&ConfigWrapper) -> KResult<Rc<dyn LcdChip>>;

fn lcd_chips() -> [(&'static str, ChipFactory); 8] {
    [
        ("st7920", st7920::new_st7920),
        ("emulated_st7920", st7920::new_emulated_st7920),
        ("hd44780", hd44780::new_hd44780),
        ("uc1701", uc1701::new_uc1701),
        ("ssd1306", uc1701::new_ssd1306),
        ("sh1106", uc1701::new_sh1106),
        ("hd44780_spi", hd44780_spi::new_hd44780_spi),
        ("aip31068_spi", aip31068_spi::new_aip31068_spi),
    ]
}

/// the framebuffer diff of the lcd chips' flush(): [pos, count] runs of
/// changed bytes, merged when `pos + gap >= nextpos` and `nextcount < 16`
pub(crate) fn framebuffer_diffs(new_data: &[u8], old_data: &[u8], gap: usize) -> Vec<(usize, usize)> {
    // Find the position of all changed bytes in this framebuffer
    let mut diffs: Vec<(usize, usize)> = new_data.iter().zip(old_data).enumerate().filter(|(_, (n, o))| n != o).map(|(i, _)| (i, 1)).collect();
    // Batch together changes that are close to each other
    if diffs.len() >= 2 {
        for i in (0..diffs.len() - 1).rev() {
            let (pos, _) = diffs[i];
            let (nextpos, nextcount) = diffs[i + 1];
            if pos + gap >= nextpos && nextcount < 16 {
                diffs[i].1 = nextcount + (nextpos - pos);
                diffs.remove(i + 1);
            }
        }
    }
    diffs
}

/// python's bytearray slice assignment ba[pos:pos+len(data)] = data (the
/// callers never write past the end)
pub(crate) fn set_slice(buf: &mut [u8], pos: usize, data: &[u8]) {
    let end = (pos + data.len()).min(buf.len());
    buf[pos..end].copy_from_slice(&data[..end - pos]);
}

pub struct PrinterLCD {
    me: Weak<PrinterLCD>,
    printer: Rc<Printer>,
    reactor: Rc<Reactor>,
    lcd_chip: Rc<dyn LcdChip>,
    menu: RefCell<Option<Rc<menu::MenuManager>>>,
    templates: RefCell<Option<Rc<PrinterDisplayTemplate>>>,
    show_data_group: RefCell<String>,
    screen_update_timer: RefCell<Option<Timer>>,
    redraw_request_pending: Cell<bool>,
    redraw_time: Cell<f64>,
}

const CMD_SET_DISPLAY_GROUP_HELP: &str = "Set the active display group";

impl PrinterLCD {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        // Load low-level lcd handler
        let chips = lcd_chips();
        let factory = config.getchoice("lcd_type", &chips, Dflt::Required)?;
        let lcd_chip = factory(config)?;
        let name = config.get_name().to_string();
        let lcd = Rc::new_cyclic(|me| PrinterLCD {
            me: me.clone(),
            printer: printer.clone(),
            reactor: reactor.clone(),
            lcd_chip: lcd_chip.clone(),
            menu: RefCell::new(None),
            templates: RefCell::new(None),
            show_data_group: RefCell::new(String::new()),
            screen_update_timer: RefCell::new(None),
            redraw_request_pending: Cell::new(false),
            redraw_time: Cell::new(0.0),
        });
        // Load menu and display_status
        if name == "display" {
            // only load menu for primary display
            *lcd.menu.borrow_mut() = Some(menu::MenuManager::new(config, &lcd)?);
        }
        printer.load_object(config, "display_status", true)?;
        // Configurable display
        let templates = lookup_display_templates(config)?;
        *lcd.templates.borrow_mut() = Some(templates.clone());
        lcd.lcd_chip.set_glyphs(&templates.display_glyphs);
        let mut dgroup = "_default_16x4".to_string();
        if lcd.lcd_chip.get_dimensions().0 == 20 {
            dgroup = "_default_20x4".into();
        }
        let dgroup = config.get("display_group", dgroup.as_str())?;
        if !templates.display_data_groups.contains_key(&dgroup) {
            return Err(KError::config(format!("Unknown display_data group '{}'", dgroup)));
        }
        *lcd.show_data_group.borrow_mut() = dgroup;
        // Screen updating
        let w = lcd.me.clone();
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| match w.upgrade() {
                Some(l) => l.handle_ready(),
                None => Ok(()),
            }),
        );
        let w = lcd.me.clone();
        printer.register_event_handler(
            lcd.lcd_chip.get_mcu().get_non_critical_reconnect_event_name(),
            Rc::new(move |_| match w.upgrade() {
                Some(l) => l.lcd_chip.init(),
                None => Ok(()),
            }),
        );
        let w = lcd.me.clone();
        *lcd.screen_update_timer.borrow_mut() = Some(reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(l) => l.screen_update_event(eventtime),
                None => crate::reactor::NEVER,
            }),
            crate::reactor::NEVER,
        ));
        // Register g-code commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let w = lcd.me.clone();
        let handler: crate::gcode::Handler = Rc::new(move |g| match w.upgrade() {
            Some(l) => l.cmd_set_display_group(g),
            None => Ok(()),
        });
        gcode.register_mux_command("SET_DISPLAY_GROUP", "DISPLAY", Some(&name), handler.clone(), Some(CMD_SET_DISPLAY_GROUP_HELP))?;
        if name == "display" {
            gcode.register_mux_command("SET_DISPLAY_GROUP", "DISPLAY", None, handler, None)?;
        }
        Ok(lcd)
    }

    /// the display templates (python reads them after creating the menu)
    fn templates(&self) -> KResult<Rc<PrinterDisplayTemplate>> {
        self.templates.borrow().clone().ok_or_else(|| KError::Other("'PrinterLCD' object has no attribute 'display_templates'".into()))
    }

    pub fn get_dimensions(&self) -> (i64, i64) {
        self.lcd_chip.get_dimensions()
    }

    fn handle_ready(&self) -> KResult<()> {
        self.lcd_chip.init()?;
        // Start screen update timer
        if let Some(t) = self.screen_update_timer.borrow().as_ref() {
            self.reactor.update_timer(t, NOW);
        }
        Ok(())
    }

    // Screen updating
    fn screen_update_event(&self, eventtime: f64) -> f64 {
        match self.screen_update(eventtime) {
            Ok(t) => t,
            Err(e) => {
                logging::exception_err("Unhandled exception in display screen update", &e);
                eventtime + REDRAW_TIME
            }
        }
    }

    fn screen_update(&self, eventtime: f64) -> KResult<f64> {
        if self.redraw_request_pending.get() {
            self.redraw_request_pending.set(false);
            self.redraw_time.set(eventtime + REDRAW_MIN_TIME);
        }
        self.lcd_chip.clear();
        // update menu component
        let menu = self.menu.borrow().clone();
        if let Some(menu) = menu {
            if menu.screen_update_event(eventtime)? {
                self.lcd_chip.flush()?;
                return Ok(eventtime + REDRAW_TIME);
            }
        }
        // Update normal display
        if let Err(e) = self.show_group(eventtime) {
            logging::exception_err("Error during display screen update", &e);
        }
        self.lcd_chip.flush()?;
        if self.redraw_request_pending.get() {
            return Ok(self.redraw_time.get());
        }
        Ok(eventtime + REDRAW_TIME)
    }

    /// DisplayGroup.show(display, templates, eventtime)
    fn show_group(&self, eventtime: f64) -> KResult<()> {
        let templates = self.templates()?;
        let group_name = self.show_data_group.borrow().clone();
        let group = &templates.display_data_groups[&group_name];
        if group.data_items.is_empty() {
            return Err(KError::Other("list index out of range".into()));
        }
        let context: Rc<RefCell<IndexMap<String, Value>>> = Rc::new(RefCell::new(create_template_context(&self.printer, Some(eventtime))));
        context.borrow_mut().insert("draw_progress_bar".into(), Value::object(DrawProgressBar { lcd: self.me.clone() }));
        context.borrow_mut().insert("render".into(), Value::object(RenderTemplate { templates: templates.clone(), context: Rc::downgrade(&context) }));
        let r = (|| -> KResult<()> {
            for (row, col, template) in &group.data_items {
                let ctx = context.borrow().clone();
                let text = template.render(Some(ctx))?;
                self.draw_text(*row, *col, &text.replace('\n', ""), eventtime);
            }
            Ok(())
        })();
        context.borrow_mut().clear(); // Remove circular references for better gc
        r
    }

    pub fn request_redraw(&self) {
        if self.redraw_request_pending.get() {
            return;
        }
        self.redraw_request_pending.set(true);
        if let Some(t) = self.screen_update_timer.borrow().as_ref() {
            self.reactor.update_timer(t, self.redraw_time.get());
        }
    }

    pub fn draw_text(&self, row: i64, col: i64, mixed_text: &str, _eventtime: f64) -> i64 {
        let mut pos = col;
        for (i, text) in mixed_text.split('~').enumerate() {
            if i & 1 == 0 {
                // write text
                self.lcd_chip.write_text(pos, row, text.as_bytes());
                pos += text.chars().count() as i64;
            } else {
                // write glyph
                pos += self.lcd_chip.write_glyph(pos, row, text);
            }
        }
        pos
    }

    pub fn draw_progress_bar(&self, row: i64, col: i64, width: i64, value: f64) -> String {
        // (python ints are unbounded; the bar is at most 16 chars = 128 bits)
        let shift = (width as f64 * 8.0 * (1.0 - value) + 0.5) as i64;
        let mut pixels: i128 = if shift >= 127 { 0 } else { -1i128 << shift.max(0) };
        pixels |= (1i128 << (width * 8 - 1)) | 1;
        for i in 0..width {
            let b = ((pixels >> (i * 8)) & 0xFF) as u8;
            let mut data = vec![0xFF];
            data.extend(std::iter::repeat_n(b, 14));
            data.push(0xFF);
            self.lcd_chip.write_graphics(col + width - 1 - i, row, &data);
        }
        String::new()
    }

    fn cmd_set_display_group(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let group = gcmd.get("GROUP")?;
        if !self.templates()?.display_data_groups.contains_key(&group) {
            return Err(gcmd.error(format!("Unknown display_data group '{}'", group)));
        }
        *self.show_data_group.borrow_mut() = group;
        Ok(())
    }
}

/// the draw_progress_bar of the display_data context
struct DrawProgressBar {
    lcd: Weak<PrinterLCD>,
}

impl Object for DrawProgressBar {
    fn type_name(&self) -> String {
        "method".into()
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn call(&self, _ctx: &mut dyn CallCtx, args: Vec<Value>, _kwargs: IndexMap<String, Value>) -> Option<VResult<Value>> {
        let lcd = self.lcd.upgrade()?;
        let num = |i: usize| args.get(i).and_then(|v| v.as_f64());
        let (Some(row), Some(col), Some(width), Some(value)) = (num(0), num(1), num(2), num(3)) else {
            return Some(Err(Exc::type_error("draw_progress_bar() missing required positional arguments")));
        };
        Some(Ok(Value::str(&lcd.draw_progress_bar(row as i64, col as i64, width as i64, value))))
    }
}

/// the render(name, **kwargs) of the display_data context
struct RenderTemplate {
    templates: Rc<PrinterDisplayTemplate>,
    context: Weak<RefCell<IndexMap<String, Value>>>,
}

impl Object for RenderTemplate {
    fn type_name(&self) -> String {
        "function".into()
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn call(&self, _ctx: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> Option<VResult<Value>> {
        let name = args.first().map(|v| v.to_str()).unwrap_or_default();
        let Some(t) = self.templates.display_templates.get(&name) else {
            return Some(Err(Exc::key_error(&Value::str(&name))));
        };
        let context = self.context.upgrade().map(|c| c.borrow().clone()).unwrap_or_default();
        Some(t.render(&context, &kwargs).map(|s| Value::str(&s)).map_err(kerror_exc))
    }
}

impl PrinterObject for PrinterLCD {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        if let Some(m) = self.menu.borrow_mut().take() {
            m.teardown();
        }
        if let Some(t) = self.screen_update_timer.borrow_mut().take() {
            self.reactor.forget_timer(&t);
        }
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterLCD::new(config)?)
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    if !config.has_section("display") {
        return Err(KError::config("A primary [display] section must be defined in printer.cfg to use auxilary displays"));
    }
    let name = config.get_name().split_whitespace().last().unwrap_or_default();
    if name == "display" {
        return Err(KError::config("Section name [display display] is not valid. Please choose a different postfix."));
    }
    Ok(PrinterLCD::new(config)?)
}
