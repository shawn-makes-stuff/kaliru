# Python plugins in kaliru — design (2026-10-24, draft)

Requirement (user, 2026-10-24): **if a plugin works in Kalico, it works in kaliru.** Full plugin support, not a
subset. Goal: kaliru can run Python klippy plugins *in general*, as Kalico can, even though the user means to move
off them over time. Their printer needs AFC (AFC-Klipper-Add-On) and the AFC_ACE plugin. Memory rule:
- no cost when no plugin is configured;
- no more than Python Kalico costs when one is.

## What Kalico offers a plugin (the contract to match)

Source: Kalico 5b59427 `klippy/printer.py` (`_load_modules`, `PrinterModule`, `load_object`), `klippy/compat.py`,
`docs/Code_Overview.md` "Adding a host module", `docs/Kalico_Additions.md` "Plugins".

1. **Discovery.** At startup Kalico lists every module in `klippy/extras/` and then `klippy/plugins/` (pkgutil). A
   name found in both is a config error, unless `[danger_options] allow_plugin_override: True`; then the plugin
   replaces the built-in extra.
2. **Loading.** A section `[name]` or `[name suffix]` imports the module on first use and calls `load_config(config)`
   or `load_config_prefix(config)`. The returned object goes into `printer.objects`. The import is lazy, and an
   import failure is logged and re-raised only when the section is used.
3. **Components.** `register_components(collector)` is called for any module whose source contains that name
   (`printer.lookup_components()`, used for load-cell and eddy sensors).
4. **Legacy imports.** `compat.install()` adds a meta-path finder, so `import mcu`, `from configfile import error`
   and `from extras.x import y` resolve to the `klippy.*` modules. The same module objects are reused, so `is` and
   `isinstance` checks hold.
5. **Everything else is unrestricted.** There is no API boundary. A plugin gets the real Python objects and can:
   - call any method (the documented set: config getters, `get_printer`, `lookup_object`, `register_event_handler`,
     `register_command`/`register_mux_command`, `pins.setup_pin`, the reactor timers/pause/fd/completions,
     `get_status`, webhooks endpoints);
   - read and write private attributes, and monkeypatch methods;
   - construct core classes itself;
   - call the C helper (`chelper.get_ffi()`) directly.
   
   Code_Overview only *asks* plugins to avoid internals.
6. **Firmware side.** `src/extras/<module>/` with its own Kconfig and Makefile is auto-included in the MCU build.
   This is out of scope for the host bridge and is tracked separately.

What the user's plugins actually do (static scan of AFC@CC1 and AFC_ACE@CC1, copies in
`Originals_Base/addons/`). AFC is the stress case for point 5:
- 62 `register_mux_command`, 34 `register_command`, 34 `register_event_handler`, 17 `register_timer`,
  12 `register_buttons`, 8 `setup_pin`, `register_endstop`, `register_lookahead_callback`, `alloc_command_queue`,
  `register_response`.
- `chelper.get_ffi()`: `trapq_alloc`/`trapq_append`/`trapq_finalize_moves`, `cartesian_stepper_alloc`,
  `itersolve_get_commanded_pos` and `set_thread_name`. AFC drives its own steppers like `manual_stepper`.
- It builds a `filament_switch_sensor` from Python (`configfile.ConfigWrapper(printer, RawConfigParser, {}, name)` then
  `printer.load_object`) and then rewires that core object:
  - `runout_helper.note_filament_present` is replaced, after `inspect.signature` on the original;
  - `runout_helper._runout_event_handler` is replaced;
  - `runout_gcode = 1` and `insert_gcode = None` are set;
  - `printer.objects["_" + name] = printer.objects.pop(name)`.
- Other internals it uses: `toolhead._advance_move_time`, `webhooks._remote_methods`, `gcode.ready_gcode_handlers`,
  `config.fileconfig.sections/options/set`, `motion_queuing.allocate_trapq/lookup_trapq_append/wipe_trapq/
  note_mcu_movequeue_activity/drip_update_time`.
- Imports of Kalico Python code it runs itself: `extras.led`, `force_move.calc_move_time`,
  `queuelogger.QueueListener/QueueHandler`, `webhooks.GCodeHelper`.
- Python features: threads (AFC logger, ACE serial), pyserial, `urllib`, `subprocess`, `inspect`, `typing`, `enum`.

## Interpreter: CPython, loaded only when a plugin is configured

| | CPython (system libpython) | RustPython (already in kaliru for python macros) |
|---|---|---|
| Compatibility | the real thing: C extensions (cffi, pyserial's termios, numpy), `inspect.signature`, threads, `match` | no `match` (0.4), older stdlib, no C extensions → no cffi, no numpy; plugins like shaketune impossible |
| Memory | measured below | heavier object model (unmeasured with AFC: it cannot import it) |
| On the CC1 | `/usr/lib/libpython3.15.so.1.0` (4.7 MB, shared) + cffi 2.0.0, pyserial 3.5, greenlet 3.4.0 installed | built in |
| Zero cost when unused | yes: bridge is a separate `.so` dlopen'ed on the first plugin section | interpreter is created lazily already |

**Decision (recommended): CPython via a bridge library** (`libkaliru_py.so`, Rust + the CPython C API, linked against
the host's libpython). kaliru `dlopen`s it only when a config section names a module that is not built in. Hosts
without Python keep working, minus plugins.

## Memory budget (measured, x86_64, CPython 3.13, module import only)

| step | own (anon) memory |
|---|---|
| bare interpreter | 4.1 MB |
| + Kalico's Python core (configfile, gcode, reactor, pins, mcu, stepper, toolhead, webhooks, queuelogger, chelper, printer) | 21.1 MB (+17.0) |
| + extras led/force_move/heaters | 26.1 MB (+5.0) |
| + AFC, 32 modules | 34.9 MB (+8.8) |
| + AFC_ACE | 36.4 MB (+1.5) |

For comparison, Python Kalico in the live scenario is 34 MB anon and kaliru is 2.7 MB. Consequences:
- The bridge must **not** load Kalico's Python core: that alone costs what Python klippy costs. Plugins talk to
  kaliru's Rust objects through thin Python-visible objects.
- Target with AFC + AFC_ACE: kaliru (about 3) + interpreter (4) + plugin code (10.3) + bridge, roughly 18-20 MB.
  That is under even plugin-free Python Kalico (34 MB); the fair comparison is Python Kalico *with* AFC, about
  34 + 10. To re-measure on armv7, where pointers are half the size.

## Architecture

1. **Module namespace.** The bridge provides a `klippy` package that matches Kalico's layout: `klippy.printer`,
   `klippy.mcu`, `klippy.extras.<name>` and so on, with `compat.install()` semantics. Each core module is a thin
   *native* module that exposes the Rust class, function or exception under its Kalico name:
   - `configfile.error`, `gcode.CommandError`, `printer.command_error`;
   - `ConfigWrapper`, `PrinterStepper`, `calc_move_time`, `LEDHelper`, and so on.
   
   Pure-Python stdlib-only helpers (queuelogger's handlers) can be Kalico's own source, imported only when a plugin
   imports them.
2. **Objects.** Each Rust printer object is exposed as a Python object of a class named like Kalico's, with the
   same methods and attributes. Identity is stable: one Python wrapper per Rust object, so `is` holds.
   Python-created objects (plugin objects) live in `printer.objects` next to the Rust ones. On the Rust side they
   implement `PrinterObject` (`get_status`, `stats`, events) by calling into Python.
3. **Calls both ways.** Rust calls Python for:
   - gcode handlers, event handlers, timers, fd callbacks, webhooks endpoints;
   - `get_status`, button callbacks, `load_config`.
   
   Python calls Rust through the wrapper methods. Errors map both ways: the Python exception classes *are* kaliru's
   classes (`KError::Py(class)` ↔ the same Python class objects).
4. **Full mirror, not a subset.** Plugins may touch anything (contract point 5), so the layer exposes *all* of
   Kalico's Python surface, generated from Kalico's source and checked for completeness:
   - **Every name.** Every module, class, function, method and attribute of Kalico 5b59427's klippy exists in the
     layer, with the same name and the same behaviour. A tool (Kalico AST vs layer) lists anything missing, and that
     list must be empty.
   - **Attributes are live views of Rust state.** Reads see the current value. Writes change it. A write of a type
     the Rust field cannot hold (AFC's `runout_gcode = 1` where Kalico holds a template) is kept as a Python value,
     and kaliru reads the field through an accessor that sees it. Lists and dicts (`printer.objects`,
     `gcode.ready_gcode_handlers`, `webhooks._remote_methods`) are live mutable views.
   - **Every method is overridable.** Kalico dispatches each `obj.method()` dynamically. Each kaliru method that
     mirrors a Python method therefore checks an override slot on entry (generated by a macro), covering:
     - an instance attribute assignment (`runout_helper.note_filament_present = f`);
     - class patching (`heaters.Heater.set_temp = f`);
     - module-function patching;
     - Python subclasses of core classes.
     
     Because the check is in the callee, every call site honours it, kaliru's own internal calls included.
   - **Core classes are constructible from Python** (`ConfigWrapper(printer, fileconfig, ...)`, `PrinterStepper`,
     `load_object` with a Python-built config), and `isinstance` against them works.
   - **Structural differences** where kaliru merged or split Kalico classes (e.g. mcu.py's helper classes are
     methods in mcu.rs) get adapter objects that rebuild Kalico's object graph. The completeness tool finds them.
5. **Greenlets.** kaliru's reactor greenlets are corosensei coroutines. Python frames can be live on several of them
   (a plugin gcode handler that waits, while a timer runs another plugin callback). Plan:
   - one `PyThreadState` per coroutine that runs Python, swapped with `PyThreadState_Swap` on every switch;
   - on the same switch, `PyUnstable_ThreadState_SetStackProtection(tstate, stack, size)` (CPython ≥ 3.15; docs:
     "When the stack is changed, for example using context switching techniques like ... boost::context, you must
     call" it).
   
   **Unverified, so it is the first prototype.** Before 3.15, CPython cannot be told about the stack. The fallback
   is to do what greenlet does or to require 3.15. The CC1 has 3.15.
6. **GIL and threads.** The reactor thread holds the GIL only while it runs Python. Plugin threads (AFC logger, ACE
   serial reader) run in between and return to the reactor via `register_async_callback`, which kaliru already has.
7. **chelper.** `chelper.get_ffi()` returns a real cffi `(ffi, lib)`, with Kalico's cdefs, over C-ABI exports of
   kaliru's chelper (`trapq_alloc`, `itersolve_*`, ...). The pointers are handles that the Rust wrappers
   (`set_trapq`, `set_stepper_kinematics`) map back to Rust objects.
8. **Override of built-in extras** (`allow_plugin_override`). Kaliru looks up core extras by concrete Rust type in
   ~400 places (`lookup::<GCodeDispatch>` 122, `ToolHead` 117, `PrinterHeaters` 12, `GCodeMove` 10, ...). A Python
   replacement gets a Rust *shell* of the replaced type whose every method override points at the Python object
   (point 4). Typed lookups keep working, and every call lands in Python. Cross-module field reads go through the
   same accessors. gcode, toolhead, pins, mcu and configfile are core files, not extras, in Kalico too, so plugins
   cannot override them there either.

## What "works in Kalico" cannot cover

These are environment differences, not API gaps:
- **Timing.** kaliru is faster, so a plugin that depends on how long Kalico's Python takes behaves as it would on
  faster hardware.
- **Files on disk.** A plugin that reads Kalico's source files or the klippy directory (`__file__` paths,
  `.version`, git describe) needs them present. Install Kalico's klippy tree as data (disk only, never imported
  unless a plugin imports a pure-Python module from it).
- **CPython version.** A plugin needs the host's CPython to run it, as it does under Kalico.

Cost of the overridable methods without plugins: one flag check per mirrored call, about one load and one branch.
The suite CPU is measured before and after. A cargo feature can compile the hooks out for builds that never load
plugins.

## How the surface is found and verified

- **Corpus:** AFC + AFC_ACE first. Then widely used plugins: Happy Hare, led_effect, klipper_tmc_autotune,
  Klippain shaketune, gcode_shell_command, z_calibration, beacon, cartographer. Each pinned to a commit in
  `Originals_Base/addons/`.
- **Completeness:** the AST surface diff (point 4) proves every Kalico name exists.
- **Behaviour:** run the stock suite and the custom cases through the layer, i.e. a test plugin that re-registers or
  wraps core objects. Each corpus plugin is also traced under Python Kalico (instrumented `__getattribute__`/
  `__setattr__` around core objects) to make sure the paths it really uses are covered by a parity test.
- **Parity:** the existing harnesses (live_cmp with linux MCU processes, case/suite) run each plugin config on
  Python Kalico and on kaliru+bridge. The serialqueue trace and the masked log must match.

## Step 1 results: prototype (`pybridge/`, branch `plugins`, 2026-10-24)

`pybridge/src/bin/proto.rs` embeds CPython 3.15.1 (uv build, `PYTHON_LIBDIR`) and runs Python on two kaliru
greenlets. `pybridge/src/greenlets.rs` is the switch hook; `src/greenlet.rs` gained `set_switch_hook` and
`Greenlet::stack()`.

| check | result |
|---|---|
| g1 suspended 50 Python frames deep inside an `except` block; g2 runs, then switches back | OK: frames, return values (55, 307) and g1's `sys.exception()` intact |
| g2 does not see g1's exception state | OK |
| C recursion (`list(map(f, ...))`, recursion limit 10^7) on a 1 MiB greenlet stack | RecursionError, no crash |
| Python thread while kaliru holds the GIL (busy 50 ms) vs while it is released (poll) | thread never ran in the held window, finished in the released one |
| **control:** no switch hook (one thread state for all greenlets) | segfault |
| **control:** swap thread states but no `SetStackProtection` | segfault |
| armv7 (cross-built, qemu-arm) | same checks pass |

Both pieces are required, so **CPython ≥ 3.15 is the minimum for plugins**. Older versions would need greenlet's
version-specific tstate patching.

Costs measured on x86_64 (CPython 3.15.1):

| item | cost |
|---|---|
| interpreter start | +2.4 MB anon (1.7 → 4.1 MB), +6.8 MB file-backed (libpython, shared) |
| Python → Rust call (`METH_FASTCALL`, no args) | 0.063 µs (a Python → Python call: 0.054 µs) |
| Rust → Python call (`PyObject_CallNoArgs`) | 0.024 µs |
| thread-state swap per greenlet switch (Enter + Main) | 0.093 µs |

On the CC1 itself (user's OK, printer idle; binary in /data/kt, removed afterwards; COSMOS's own CPython 3.15.0,
Cortex-A7):

| item | cost |
|---|---|
| all checks | greenlets OK, thread OK; `noswap` control segfaults (exit 139) |
| interpreter start | +1.7 MB anon (0.4 → 2.1 MB), +3.1 MB file-backed |
| after the greenlet and thread tests | 4.2 MB anon in total (includes the 1 MiB greenlet stack the C-recursion test touched) |
| Python → Rust call | 1.94 µs (a Python → Python call: 1.60 µs) |
| Rust → Python call | 0.82 µs |
| thread-state swap per greenlet switch | 2.5 µs |

qemu-arm numbers are not used: qemu's own RSS is counted, and the timing is emulation speed.

The GIL model was tested as designed:
- the reactor thread keeps the GIL except while it blocks in poll;
- while Python bytecode runs, CPython's 5 ms switch interval lets threads run, as in Kalico.

Plugin threads therefore run only at points where they could also run under Kalico.

## Status (2026-10-24, branch `plugins`)

### AFC + AFC_ACE run identically to Kalico

`tools/plugin_cmp.sh` runs three cases, all identical to Kalico (serial trace, exit code, masked log):

| Case | What it covers |
|---|---|
| `basic` | the test plugin |
| `afc_ace` | AFC + AFC_ACE start-up through ready |
| `afc_ace_cmds` | lane settings, filament sensors, mapping, `PREP` (about 1,700 log lines), `AFC_M104`, AFC's status read from Jinja |

Both sides run on CPython 3.15 with the same packages: the reference uses `~/rk/venv315`, and kaliru gets that venv's site-packages.

There is no regression without plugins: stock suite 87/88 (`printers/13` timing, as before), cases 62/62.

### How the layer is built (`src/plugins/`)

**Python start-up**
- `capi.rs` loads libpython at run time.
- `greenlets.rs` gives each greenlet its own thread state. The greenlet Python starts in keeps the starting thread state.
- `gil.rs` holds the layer's tables as GIL-guarded statics.

**The mirror**
- `tools/gen_mirror.py` generates `mirror/*.py` and `mirror.bin` from Kalico. It covers 208 modules and 7,489 IDs; 39 methods are copied verbatim (`tools/mirror_verbatim.txt`): all of ConfigWrapper and GCodeCommand, and mathutil.
- `prelude.py` provides:
  - Object, Meta and Attr. An object with no kaliru object behind it is plain Python.
  - The import finder and compat aliases.
  - Logging to klippy.log.
  - Live views: FileConfig, AccessTracking, ObjectsView, ListView and DictView.
  - `overridden()`.

**Bridge to kaliru**
- `native.rs` holds the `_kaliru` module, the handler registry, and the List/Dict fields.
- The BYPASS stack lets a plugin's replacement call the original method.
- `objects.rs` keeps one Python object per kaliru object. The class is found from the Rust type path. Casters keep printer objects usable when they reach Python as a plain object.
- `py.rs` converts errors both ways (`KError::Foreign` keeps the Python exception and its traceback). `convert.rs` holds ToPy/FromPy and the method!/function!/cell_attr!/ro_attr!/vec_attr! macros.
- `dispatch()` lets kaliru methods honour a plugin's replacement. It is wired for `RunoutHelper.note_filament_present` and `_runout_event_handler`.

**Handlers (`impls/`)**

printer, configfile, gcode, reactor, webhooks, pins (Python pin chips), buttons, heaters (plugin heaters in `heaters`), filament_switch_sensor, gcode_move, idle_timeout, heaters.Heater, toolhead, PrinterExtruder.

**Supporting changes in kaliru's core**

| Change | Where |
|---|---|
| `running.rs` | new: the Rc of objects passed to callbacks by reference |
| `PrinterObject::type_name` / `call_method` | `printer.rs` |
| `HeaterSlot` | `heaters.rs` |
| `GcodeSlot`; settable RunoutHelper fields | `filament_switch_sensor.rs` |
| output callback origins; handler-dict accessors | `gcode.rs` |
| `PinChip::register_config_callback` / `python_object` | `pins.rs` |
| `pub(crate)` fields | gcode_move, idle_timeout, Heater, ToolHead, PrinterExtruder |

### Known gaps, all of which fail loudly

- **Coverage.** Most of the 7,489 IDs still raise "kaliru: X is not mirrored yet". The work list is the surface tool plus a trace per corpus plugin.
- **Override dispatch.** It is wired only where plugins are known to patch. Every other kaliru method still runs its own code.
- **Str enums in Jinja.** A str enum in a status (AFC's `State`) reaches Jinja and JSON as its value. JSON matches Kalico, but a macro that renders the enum directly gets `Idle`, where Kalico shows `State.IDLE`.
- **Valueless options.** A plugin-built `RawConfigParser` option with no value (`set(section, option)`) reads back as "" in kaliru.
- **Output handlers.** An exception raised by a Python output handler is logged instead of propagated.
- **Not supported yet:**
  - waiting on a plugin's heater (`set_temperature(..., wait=True)`);
  - a Python extra axis (`set_extruder` with a plugin object);
  - assigning into the gcode handler dicts;
  - AFC_stepper lanes (chelper ffi, PrinterStepper);
  - SHAPER/TMC/MCU command objects.
- **DeprecationWarning.** The CPython debug build prints `linecache` "`__loader__ != __spec__.loader`" for mirror modules. That points at module attributes worth checking.

## Steps

1. Prototype the risky core and verify it before anything else (WSL, CPython 3.15 built locally):
   - dlopen'ed bridge;
   - a plugin gcode command that pauses while a timer runs Python in another coroutine (tstate swap + stack
     protection);
   - a thread calling `register_async_callback`;
   - one overridable Rust method patched from Python and called from Rust;
   - memory, crossing cost and startup, on x86 and qemu-arm.
2. Surface tool: Kalico AST inventory (modules, classes, methods, attributes, module functions) and the layer diff.
3. Loader parity: discovery of extras and plugins, `allow_plugin_override` (shells), lazy import,
   `register_components`, compat finder. Cases against Kalico.
4. Mirror generation, module by module (printer, configfile, reactor, gcode, pins, mcu, stepper, toolhead, chelper
   ffi, then extras). The diff list shrinks to empty, and each module gets parity cases.
5. AFC + AFC_ACE with the user's config (copied read-only from the CC1), then the corpus plugins.
6. Later, decide whether python *macros* move from RustPython to the same bridge. Without RustPython the binary
   shrinks from 29.7 to 9.1 MB, but macros would then need libpython.
