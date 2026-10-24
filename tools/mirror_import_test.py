#!/usr/bin/env python3
# Imports every generated mirror module (src/plugins/mirror) through the real
# prelude (src/plugins/prelude.py) with a stub native module, to check that
# all of them evaluate (definition-time names, bases, defaults) under the
# CPython that will run them. Then checks the plugin-facing behaviour the
# prelude promises: signatures, legacy imports, subclass/patch reports.
#
# usage: python3.15 mirror_import_test.py KALIRU_DIR
import inspect
import os
import sys
import types

root = sys.argv[1]
mirror = os.path.join(root, "src/plugins/mirror")
modules = {}
for line in open(os.path.join(root, "src/plugins/mirror_ids.rs")):
    line = line.strip()
    if line.startswith('("'):
        name, pkg = line.strip("(),").split(", ")
        modules[name.strip('"')] = pkg == "true"

reports = []
stub = types.ModuleType("_kaliru")
stub.call = stub.fcall = stub.new = lambda *a: None
stub.get = lambda obj, id: None
stub.set = stub.delete = lambda *a: None
stub.patched_class = lambda cls, name: reports.append(("class", cls.__name__, name))
stub.patched_instance = lambda obj, name: reports.append(("inst", type(obj).__name__, name))
stub.source = lambda name: open(os.path.join(mirror, name + ".py")).read()


def module_value(modname, name):
    raise AttributeError("module %r has no attribute %r" % (modname, name))


stub.module_value = module_value
sys.modules["_kaliru"] = stub
prelude = types.ModuleType("_k")
exec(compile(open(os.path.join(root, "src/plugins/prelude.py")).read(), "prelude.py", "exec"), prelude.__dict__)
sys.modules["_k"] = prelude
klippy_dir = os.path.join(root, "tools/mirror_test_klippy")
prelude.install(klippy_dir, modules, 20)

import importlib
failed = 0
for name in sorted(modules):
    try:
        importlib.import_module(name)
    except Exception as e:
        failed += 1
        print("FAIL", name, type(e).__name__, e)
print("imported %d/%d mirror modules" % (len(modules) - failed, len(modules)))

# plugin-facing checks
from klippy.extras import filament_switch_sensor as fss
sig = list(inspect.signature(fss.RunoutHelper.note_filament_present).parameters)
print("signature", "OK" if sig == ["self", "eventtime", "is_filament_present", "force", "immediate"] else sig)
import mcu, configfile  # legacy imports (Kalico compat)
import klippy.mcu
print("compat alias", "OK" if mcu is klippy.mcu and configfile.error is klippy.configfile.error else "FAIL")
from extras.heaters import Heater
reports.clear()


class MyHeater(Heater):
    def set_temp(self, degrees):
        pass


Heater.get_temp = lambda self, t: (0, 0)
obj = object.__new__(MyHeater)
object.__setattr__(obj, "_k_h", 1)  # as if kaliru had bound it
obj.check_busy = lambda *a: False
ok = ("class", "MyHeater", "set_temp") in reports \
    and ("class", "Heater", "get_temp") in reports and ("inst", "MyHeater", "check_busy") in reports
print("patch reports", "OK" if ok else reports)
print("isinstance", "OK" if isinstance(obj, Heater) and issubclass(MyHeater, klippy.extras.heaters.Heater) else "FAIL")
