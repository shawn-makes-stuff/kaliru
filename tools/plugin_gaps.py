#!/usr/bin/env python3
# Which parts of Kalico's surface a plugin uses that kaliru's plugin layer
# does not handle yet (static estimate, see corpus_use.py): Kalico member
# names the plugin's code uses (attributes / calls on other objects) whose
# every Kalico class member of that name lacks a handler in src/plugins/impls
# and is not verbatim. Generic get_status/stats/get_temp count as handled.
#
# usage: plugin_gaps.py KALIRU_DIR PLUGIN_DIR [...]
import ast
import json
import pathlib
import re
import subprocess
import sys

root = pathlib.Path(sys.argv[1])
surface = json.loads(subprocess.run(
    [sys.executable, str(root / "pybridge/tools/surface.py"), str(root / "../Originals_Base/kalico")],
    capture_output=True, text=True).stdout)
handled = set()
for p in (root / "src/plugins/impls").glob("*.rs"):
    handled |= set(re.findall(r'"(klippy\.[A-Za-z0-9_.]+)"', p.read_text()))
    # method!(r, "klippy.x.Class", Type, name(...)) and friends
    for cls, name in re.findall(r'(?:method|cell_attr|ro_attr|vec_attr)!\(r, "(klippy\.[A-Za-z0-9_.]+)"(?:\.[^,]*)?, [^,]+, (?:r#)?([A-Za-z_0-9]+)', p.read_text()):
        handled.add(f"{cls}.{name}")
for line in (root / "tools/mirror_verbatim.txt").read_text().splitlines():
    if line.strip() and not line.startswith("#"):
        handled.add(line.strip())
members = {}
for mod, m in surface.items():
    for cname, c in m["classes"].items():
        for name in list(c["methods"]) + c["attrs"]:
            members.setdefault(name, []).append(f"{mod}.{cname}.{name}")
generic = {"get_status", "stats", "get_temp"}
for d in sys.argv[2:]:
    used, defined = set(), set()
    for path in pathlib.Path(d).rglob("*.py"):
        if "test" in path.parts or "tools" in path.parts:
            continue
        tree = ast.parse(path.read_text(errors="replace"))
        for n in ast.walk(tree):
            if isinstance(n, ast.Attribute):
                used.add(n.attr)
                if isinstance(n.ctx, ast.Store):
                    defined.add(n.attr)
            elif isinstance(n, (ast.FunctionDef, ast.ClassDef)):
                defined.add(n.name)
    gaps = []
    for name in sorted(used - defined - generic):
        quals = members.get(name)
        if quals and not any(q in handled for q in quals):
            gaps.append(name)
    print(f"{pathlib.Path(d).name}: {len(gaps)} Kalico member names without a handler")
    print("  " + " ".join(gaps))
