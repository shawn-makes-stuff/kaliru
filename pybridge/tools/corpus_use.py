#!/usr/bin/env python3
# Static estimate of which parts of Kalico's surface (surface.py output) a
# plugin uses: the klippy modules and names it imports, and the attribute
# names it uses on other objects that exist in Kalico's classes and that the
# plugin does not define itself. An over-estimate by design (a name match is
# not proof of use); tracing under Python Kalico gives the exact set.
#
# usage: corpus_use.py surface.json PLUGIN_DIR [...]
import ast
import json
import pathlib
import sys


def scan(paths, kalico_modules):
    imports, used, defined = set(), set(), set()
    for path in paths:
        try:
            tree = ast.parse(path.read_text(errors="replace"), str(path))
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module:
                mod = node.module.removeprefix("klippy.")
                for a in node.names:
                    full = mod + "." + a.name
                    if "klippy." + full in kalico_modules:
                        imports.add(full)
                    elif "klippy." + mod in kalico_modules:
                        imports.add(mod + ":" + a.name)
            elif isinstance(node, ast.Import):
                for a in node.names:
                    if "klippy." + a.name.removeprefix("klippy.") in kalico_modules:
                        imports.add(a.name.removeprefix("klippy."))
            elif isinstance(node, ast.Attribute):
                used.add(node.attr)
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                defined.add(node.name)
            if isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Store):
                defined.add(node.attr)
    return imports, used, defined


def main():
    surface = json.load(open(sys.argv[1]))
    kalico_modules = set(surface)
    members = {}
    for mod, m in surface.items():
        for cname, c in m["classes"].items():
            for name in list(c["methods"]) + c["attrs"] + c["class_attrs"]:
                members.setdefault(name, set()).add(f"{mod.removeprefix('klippy.')}.{cname}")
    total = set()
    for d in sys.argv[2:]:
        paths = [p for p in pathlib.Path(d).rglob("*.py") if "test" not in p.parts]
        imports, used, defined = scan(paths, kalico_modules)
        hits = {n for n in used - defined if n in members and not n.startswith("__")}
        private = sorted(n for n in hits if n.startswith("_"))
        total |= hits
        print(f"{pathlib.Path(d).name}: {len(paths)} files; imports {len(imports)} klippy names; "
              f"{len(hits)} Kalico member names used ({len(private)} private: {' '.join(private[:12])})")
        print("   imports:", " ".join(sorted(imports)))
    print(f"corpus total: {len(total)} distinct Kalico member names "
          f"(of {len(members)} in Kalico's classes)")


if __name__ == "__main__":
    main()
