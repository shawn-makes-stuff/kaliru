#!/usr/bin/env python3
# Inventory of Kalico's Python surface: everything a plugin can reach by
# importing klippy modules (docs/PLUGINS.md "Full mirror"). For each module:
# module-level names (functions, classes, constants, imported names), and for
# each class: bases, methods (with their parameter lists, since plugins such
# as AFC inspect signatures), class attributes and instance attributes
# (`self.x = ...` anywhere in its methods).
#
# usage: surface.py KALICO_DIR > surface.json      (summary on stderr)
import ast
import json
import pathlib
import sys


def params(fn):
    a = fn.args
    out = [p.arg for p in a.posonlyargs + a.args]
    if a.vararg:
        out.append("*" + a.vararg.arg)
    out += [p.arg for p in a.kwonlyargs]
    if a.kwarg:
        out.append("**" + a.kwarg.arg)
    return out


def targets(node):
    # names bound by an assignment target
    if isinstance(node, ast.Name):
        yield node.id
    elif isinstance(node, (ast.Tuple, ast.List)):
        for e in node.elts:
            yield from targets(e)


def class_info(cls):
    info = {"bases": [ast.unparse(b) for b in cls.bases], "methods": {}, "class_attrs": [], "attrs": set()}
    for node in cls.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            info["methods"][node.name] = params(node)
            if node.args.args:
                me = node.args.args[0].arg  # self (or cls)
                for sub in ast.walk(node):
                    if isinstance(sub, ast.Attribute) and isinstance(sub.ctx, ast.Store) \
                            and isinstance(sub.value, ast.Name) and sub.value.id == me:
                        info["attrs"].add(sub.attr)
        elif isinstance(node, (ast.Assign, ast.AnnAssign)):
            for t in (node.targets if isinstance(node, ast.Assign) else [node.target]):
                info["class_attrs"] += list(targets(t))
        elif isinstance(node, ast.ClassDef):
            info["class_attrs"].append(node.name)
    info["attrs"] = sorted(info["attrs"] - set(info["methods"]))
    return info


def module_info(tree):
    names, classes, functions = [], {}, {}
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            classes[node.name] = class_info(node)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            functions[node.name] = params(node)
        elif isinstance(node, (ast.Assign, ast.AnnAssign)):
            for t in (node.targets if isinstance(node, ast.Assign) else [node.target]):
                names += list(targets(t))
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            names += [(a.asname or a.name).split(".")[0] for a in node.names if a.name != "*"]
        elif isinstance(node, (ast.If, ast.Try)):
            # conditional definitions (try: import x / except ...)
            for sub in ast.walk(node):
                if isinstance(sub, (ast.Import, ast.ImportFrom)):
                    names += [(a.asname or a.name).split(".")[0] for a in sub.names if a.name != "*"]
                elif isinstance(sub, ast.Assign):
                    for t in sub.targets:
                        names += list(targets(t))
    return {"names": sorted(set(names)), "functions": functions, "classes": classes}


def main():
    root = pathlib.Path(sys.argv[1]) / "klippy"
    out = {}
    for path in sorted(root.rglob("*.py")):
        rel = path.relative_to(root.parent).with_suffix("")
        parts = list(rel.parts)
        if parts[-1] == "__init__":
            parts.pop()
        out[".".join(parts)] = module_info(ast.parse(path.read_text(), str(path)))
    json.dump(out, sys.stdout, indent=1, sort_keys=True)
    n_cls = sum(len(m["classes"]) for m in out.values())
    n_meth = sum(len(c["methods"]) for m in out.values() for c in m["classes"].values())
    n_attr = sum(len(c["attrs"]) + len(c["class_attrs"]) for m in out.values() for c in m["classes"].values())
    n_fn = sum(len(m["functions"]) for m in out.values())
    n_names = sum(len(m["names"]) for m in out.values())
    print(f"{len(out)} modules, {n_cls} classes, {n_meth} methods, {n_attr} class/instance attributes, "
          f"{n_fn} module functions, {n_names} other module names", file=sys.stderr)


if __name__ == "__main__":
    main()
