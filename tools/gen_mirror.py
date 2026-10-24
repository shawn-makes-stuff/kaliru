#!/usr/bin/env python3
# Generates the Python side of kaliru's plugin layer (docs/PLUGINS.md "Full
# mirror"): for every module of Kalico's klippy package a Python module with
# the same classes, methods (exact signatures and defaults), functions and
# constants, whose bodies call into kaliru through the native module `_k`.
#
#  - mirrored class: `class X(<bases>):` whose methods are
#    `def m(self, a, b=1): return _k.call(self, ID, a, b)`, `__init__` is
#    `_k.new(self, ID, ...)` and instance attributes are _k.Attr(ID) data
#    descriptors; the root base is _k.Object (metaclass _k.Meta reports
#    patches and subclass overrides to kaliru)
#  - exception classes are copied verbatim (their identity is what errors
#    are matched by on both sides)
#  - literal module constants and class attributes are copied; module names
#    bound to cheap stdlib objects (`error = configparser.Error`) are copied
#    with their import; everything else resolves on first access through the
#    module's __getattr__ (_k.module_getattr), so a mirror module never
#    imports numpy and the like up front
#
# Output: src/plugins/mirror/<module>.py (one per klippy module) and
# src/plugins/mirror_ids.rs (the ID -> qualified name table, same order).
#
# usage: gen_mirror.py KALICO_DIR OUT_DIR   (OUT_DIR = kaliru/src/plugins)
import ast
import builtins
import pathlib
import struct
import zlib
import subprocess
import sys

HEAVY = {"greenlet", "numpy", "scipy", "matplotlib", "cffi", "serial", "can", "msgspec", "jinja2", "markupsafe"}
EXC_ROOTS = {"Exception", "BaseException", "configparser.Error", "error", "CommandError",
             "ValueError", "KeyError", "RuntimeError", "IOError", "OSError", "LookupError"}


BUILTINS = set(dir(builtins))
# methods emitted as Kalico's own code (one qualified name per line)
VERBATIM = {l.split("#")[0].strip() for l in open(pathlib.Path(__file__).with_name("mirror_verbatim.txt"))} - {""}
VERBATIM_SEEN = set()


def is_main_guard(node):
    return isinstance(node, ast.If) and "__main__" in ast.unparse(node.test)


def is_docstring(node):
    return isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant)


def bound_names(node):
    """names a statement binds (plain assignment targets, also inside try/if)"""
    out = set()
    for sub in ast.walk(node):
        if isinstance(sub, ast.Name) and isinstance(sub.ctx, ast.Store):
            out.add(sub.id)
    return out


def statement_loads(node):
    """free names a statement loads"""
    loads = {n.id for n in ast.walk(node) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
    for sub in ast.walk(node):
        if isinstance(sub, ast.Lambda):
            loads -= {a.arg for a in sub.args.args + sub.args.kwonlyargs + sub.args.posonlyargs}
        elif isinstance(sub, ast.comprehension):
            loads -= {n.id for n in ast.walk(sub.target) if isinstance(n, ast.Name)}
        elif isinstance(sub, ast.ExceptHandler) and sub.name:
            loads.discard(sub.name)
    return loads - bound_names(node)


class Ids:
    def __init__(self):
        self.names = []

    def new(self, qualname):
        self.names.append(qualname)
        return len(self.names) - 1


def param_list(fn, src):
    """the def's parameter list as source text, and the call argument list"""
    a = fn.args
    parts, call = [], []
    pos = a.posonlyargs + a.args
    defaults = [None] * (len(pos) - len(a.defaults)) + list(a.defaults)
    for i, (p, d) in enumerate(zip(pos, defaults)):
        parts.append(p.arg + ("" if d is None else "=" + ast.get_source_segment(src, d)))
        call.append(p.arg)
        if a.posonlyargs and i == len(a.posonlyargs) - 1:
            parts.append("/")
    if a.vararg:
        parts.append("*" + a.vararg.arg)
        call.append(a.vararg.arg)
    elif a.kwonlyargs:
        parts.append("*")
    for p, d in zip(a.kwonlyargs, a.kw_defaults):
        parts.append(p.arg + ("" if d is None else "=" + ast.get_source_segment(src, d)))
        call.append(p.arg)
    if a.kwarg:
        parts.append("**" + a.kwarg.arg)
        call.append(a.kwarg.arg)
    return ", ".join(parts), call


def targets(node):
    if isinstance(node, ast.Name):
        yield node.id
    elif isinstance(node, (ast.Tuple, ast.List)):
        for e in node.elts:
            yield from targets(e)


def names_in(node):
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}


def is_literal(node):
    try:
        ast.literal_eval(node)
        return True
    except Exception:
        return False


def is_exception_class(cls, exc_names):
    for b in cls.bases:
        t = ast.unparse(b)
        if t in EXC_ROOTS or t in exc_names or t.split(".")[-1] in exc_names:
            return True
    return False


class ModuleGen:
    def __init__(self, modname, src, ids):
        self.modname = modname
        self.src = src
        self.tree = ast.parse(src)
        self.ids = ids
        self.imports = {}       # bound name -> import statement source
        self.stdlib_names = set()
        self.heavy_names = set()
        self.stdlib_classes = set()  # names bound by stdlib imports (enum, typing, ...)
        self.star_imports = []
        self.verbatim_used = []
        self.out = []
        self.needed = set()     # names the emitted code references at definition time
        self.bound = set()      # module constants emitted so far

    def collect_imports(self):
        for node in self.tree.body:
            stmts = [node]
            if isinstance(node, (ast.If, ast.Try)):
                stmts = [n for n in ast.walk(node) if isinstance(n, (ast.Import, ast.ImportFrom))]
            for n in stmts:
                if isinstance(n, ast.Import):
                    for a in n.names:
                        bound = a.asname or a.name.split(".")[0]
                        self.imports[bound] = ast.unparse(ast.Import(names=[a]))
                        if a.name.split(".")[0] not in HEAVY:
                            self.stdlib_names.add(bound)
                            self.stdlib_classes.add(bound)
                        else:
                            self.heavy_names.add(bound)
                elif isinstance(n, ast.ImportFrom):
                    for a in n.names:
                        if a.name == "*":
                            # klippy/__init__.py: from .printer import * (mirror modules: cheap)
                            if n.level > 0:
                                self.star_imports.append(ast.unparse(n))
                            continue
                        bound = a.asname or a.name
                        self.imports[bound] = ast.unparse(ast.ImportFrom(module=n.module, names=[a], level=n.level))
                        if n.level == 0 and (n.module or "").split(".")[0] not in HEAVY | {"klippy"}:
                            self.stdlib_names.add(bound)
                            self.stdlib_classes.add(bound)
                        elif n.level > 0 or (n.module or "").startswith("klippy"):
                            self.stdlib_names.add(bound)

    def is_data_statement(self, node):
        """an assignment or a call statement that only uses builtins, stdlib
        and mirror imports, and names bound by earlier data statements"""
        if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
            if getattr(node, "value", None) is None:
                return False
            new = set(self.assigned_names(node)) if not isinstance(node, ast.AugAssign) else set()
        elif isinstance(node, ast.Expr) and isinstance(node.value, ast.Call):
            new = set()
        else:
            return False
        loads = {n.id for n in ast.walk(node) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
        # names bound inside the statement itself: lambda parameters, comprehension targets
        for sub in ast.walk(node):
            if isinstance(sub, ast.Lambda):
                loads -= {a.arg for a in sub.args.args + sub.args.kwonlyargs + sub.args.posonlyargs}
            elif isinstance(sub, ast.comprehension):
                loads -= set(targets(sub.target))
        # subscript / attribute targets load their base name
        return loads - new <= BUILTINS | self.stdlib_names | self.bound

    @staticmethod
    def assigned_names(node):
        """plain names an assignment binds (not subscript / attribute targets)"""
        out = []
        for t in (node.targets if isinstance(node, ast.Assign) else [node.target]):
            out += list(targets(t))
        return out


    def emit_verbatim(self, fn, indent):
        """Kalico's own method body (tools/mirror_verbatim.txt): derived
        methods built on mirrored primitives (ConfigWrapper.getchoice)"""
        for dec in fn.decorator_list:
            self.out.append(indent + "@" + ast.unparse(dec))
        # the segment starts at `def`; later lines keep their original indentation
        lines = ast.get_source_segment(self.src, fn).split("\n")
        shift = len(indent) - fn.col_offset
        self.out.append(indent + lines[0])
        for l in lines[1:]:
            if not l.strip():
                self.out.append("")
            elif shift >= 0:
                self.out.append(" " * shift + l)
            else:
                self.out.append(l[-shift:])
        self.verbatim_used.append(fn)

    def emit_function(self, fn, indent, qual, method):
        if qual in VERBATIM:
            VERBATIM_SEEN.add(qual)
            self.ids.new(qual)  # IDs stay stable whichever way a method is emitted
            return self.emit_verbatim(fn, indent)
        params, call = param_list(fn, self.src)
        for d in fn.args.defaults + [d for d in fn.args.kw_defaults if d is not None]:
            self.needed |= names_in(d)
        for dec in fn.decorator_list:
            self.out.append(indent + "@" + ast.unparse(dec))
            self.needed |= names_in(dec)
        fid = self.ids.new(qual)
        kind = "def"
        if isinstance(fn, ast.AsyncFunctionDef):
            kind = "async def"
        decos = {ast.unparse(d) for d in fn.decorator_list}
        args = ", ".join(call)
        if method and fn.name == "__init__":
            body = f"_k.new({args.split(', ', 1)[0]}, {fid}{', ' + args.split(', ', 1)[1] if ', ' in args else ''})"
        elif method and not decos & {"staticmethod", "classmethod"}:
            first, _, rest = args.partition(", ")
            body = f"return _k.call({first}, {fid}{', ' + rest if rest else ''})"
        else:
            body = f"return _k.fcall({fid}{', ' + args if args else ''})"
        self.out.append(f"{indent}{kind} {fn.name}({params}):")
        self.out.append(f"{indent}    {body}")

    def emit_class(self, cls, exc_names):
        stdlib_base = any(ast.unparse(b).split(".")[0] in self.stdlib_classes for b in cls.bases)
        # marker / data classes without methods (configfile's `class sentinel`)
        plain = not cls.bases and not any(isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) for n in cls.body)
        if is_exception_class(cls, exc_names) or stdlib_base or plain:
            exc_names.add(cls.name)
            self.out.append(ast.get_source_segment(self.src, cls))
            self.needed |= names_in(cls)
            self.out.append("")
            return
        bases = [ast.unparse(b) for b in cls.bases if ast.unparse(b) != "object"]
        bases = [b for b in bases if not b.startswith(("Generic", "Protocol", "ABC"))]
        # bases from C extension packages (reactor's greenlet.greenlet): the
        # kaliru object is not one; the class roots at _k.Object instead
        bases = [b for b in bases if b.split(".")[0] not in self.heavy_names]
        for b in bases:
            self.needed |= {b.split(".")[0]}
        self.out.append(f"class {cls.name}({', '.join(bases) or '_k.Object'}):")
        self.out.append("    __kaliru_mirror__ = True")
        qual = f"{self.modname}.{cls.name}"
        attrs, methods, local = set(), set(), set()
        n0 = len(self.out)
        for node in cls.body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                methods.add(node.name)
                self.emit_function(node, "    ", f"{qual}.{node.name}", True)
                if node.args.args:
                    me = node.args.args[0].arg
                    for sub in ast.walk(node):
                        if isinstance(sub, ast.Attribute) and isinstance(sub.ctx, ast.Store) \
                                and isinstance(sub.value, ast.Name) and sub.value.id == me:
                            attrs.add(sub.attr)
            elif isinstance(node, ast.ClassDef):
                # nested helper classes (gcode's `class sentinel:`) as they are
                self.out.append("    " + ast.unparse(node).replace("\n", "\n    "))
                self.needed |= names_in(node)
                local.add(node.name)
            elif isinstance(node, (ast.Assign, ast.AnnAssign)) and node.value is not None \
                    and (is_literal(node.value)
                         or names_in(node.value) <= self.bound | local | self.stdlib_names | BUILTINS):
                self.out.append("    " + ast.unparse(node))
                self.needed |= names_in(node.value)
                for t in (node.targets if isinstance(node, ast.Assign) else [node.target]):
                    local |= set(targets(t))
        for a in sorted(attrs - methods):
            aid = self.ids.new(f"{qual}.{a}")
            self.out.append(f"    {a} = _k.Attr({aid}, {a!r})")
        self.out.append("")

    def generate(self):
        self.collect_imports()
        exc_names = set()
        body = []
        consts = []
        later = []
        for node in self.tree.body:
            if isinstance(node, ast.ClassDef):
                body.append(node)
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                body.append(node)
            elif self.is_data_statement(node):
                # module data (constants, register tables built from other
                # modules' tables: tmc2209's Fields), in source order
                consts.append(node)
                if not isinstance(node, ast.Expr):
                    self.bound |= set(self.assigned_names(node))
            elif isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign, ast.Expr, ast.Try, ast.If)) \
                    and not is_main_guard(node) and not is_docstring(node):
                later.append(node)
        for c in consts:
            self.out.append(ast.unparse(c))
            self.needed |= names_in(c)
        self.out.append("")
        for node in body:
            if isinstance(node, ast.ClassDef):
                self.emit_class(node, exc_names)
            else:
                self.emit_function(node, "", f"{self.modname}.{node.name}", False)
                self.out.append("")
            # later class attributes may name it (GCodeCommand's error = CommandError)
            self.bound.add(node.name)
        defined = {n.name for n in body}
        # statements that use the module's classes (reactor's
        # `try: select.poll; Reactor = PollReactor`), in source order
        for node in later:
            if statement_loads(node) <= BUILTINS | self.stdlib_names | self.bound | defined:
                self.out.append(ast.unparse(node))
                self.needed |= names_in(node)
                self.bound |= set(bound_names(node))
        for c in consts:
            if not isinstance(c, ast.Expr):
                defined |= set(self.assigned_names(c))
        head = [f"# kaliru plugin layer: mirror of Kalico {self.modname} (generated by tools/gen_mirror.py)",
                "import _k"]
        for name in sorted(self.needed - defined):
            if name in self.imports:
                head.append(self.imports[name])
        head += self.star_imports
        head.append("")
        module_names = set(BUILTINS) | {"_k", "__name__", "__file__"} | defined | self.bound
        # imports the head actually has
        module_names |= {n for n in self.imports if self.imports[n] in head}
        for fn in self.verbatim_used:
            local = {a.arg for a in fn.args.args + fn.args.kwonlyargs + fn.args.posonlyargs}
            local |= {fn.args.vararg.arg} if fn.args.vararg else set()
            local |= {fn.args.kwarg.arg} if fn.args.kwarg else set()
            local |= bound_names(fn) | {n.name for n in ast.walk(fn) if isinstance(n, ast.FunctionDef)}
            # parameters of nested functions and lambdas
            for sub in ast.walk(fn):
                if isinstance(sub, (ast.FunctionDef, ast.Lambda)):
                    sa = sub.args
                    local |= {x.arg for x in sa.args + sa.kwonlyargs + sa.posonlyargs}
                    local |= {x.arg for x in (sa.vararg, sa.kwarg) if x}
            # defaults and decorators are evaluated in the class body, not here
            body = ast.Module(body=fn.body, type_ignores=[])
            free = statement_loads(body) - local - module_names
            for name in sorted(free):
                if name in self.imports:
                    head.append(self.imports[name])
                    module_names.add(name)
                else:
                    raise SystemExit(f"verbatim {self.modname}.{fn.name} uses {name!r}, which the mirror module does not bind")
        tail = ["", "__getattr__ = _k.module_getattr(__name__)", ""]
        return "\n".join(head + self.out + tail)


def main():
    kalico, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    rev = subprocess.run(["git", "-C", str(kalico), "rev-parse", "--short", "HEAD"],
                         capture_output=True, text=True).stdout.strip()
    ids = Ids()
    mdir = out / "mirror"
    mdir.mkdir(parents=True, exist_ok=True)
    for old in mdir.glob("*.py"):
        old.unlink()
    modules = []
    for path in sorted((kalico / "klippy").rglob("*.py")):
        rel = path.relative_to(kalico).with_suffix("")
        parts = list(rel.parts)
        is_pkg = parts[-1] == "__init__"
        if is_pkg:
            parts.pop()
        modname = ".".join(parts)
        src = path.read_text()
        text = ModuleGen(modname, src, ids).generate()
        compile(text, modname, "exec")
        (mdir / (modname + ".py")).write_text(text)
        modules.append((modname, is_pkg))
    # mirror.bin: u32 count, then per module (u16 name length, name, u32
    # compressed length, u32 source length, zlib stream); little endian
    with open(out / "mirror.bin", "wb") as f:
        f.write(struct.pack("<I", len(modules)))
        for m, _ in modules:
            src = (mdir / (m + ".py")).read_bytes()
            z = zlib.compress(src, 9)
            f.write(struct.pack("<H", len(m)) + m.encode() + struct.pack("<II", len(z), len(src)) + z)
    with open(out / "mirror_ids.rs", "w") as f:
        f.write(f"// generated by tools/gen_mirror.py from Kalico {rev}: do not edit\n")
        f.write("/// mirrored module names; true = a package\n")
        f.write("pub static MODULES: &[(&str, bool)] = &[\n")
        for m, p in modules:
            f.write(f"    ({m!r}, {'true' if p else 'false'}),\n".replace("'", '"'))
        f.write("];\n\n/// qualified name of each method / function / attribute ID\n")
        f.write("pub static IDS: &[&str] = &[\n")
        for n in ids.names:
            f.write(f'    "{n}",\n')
        f.write("];\n")
    missing = VERBATIM - VERBATIM_SEEN
    if missing:
        raise SystemExit(f"mirror_verbatim.txt names Kalico does not have: {sorted(missing)}")
    print(f"{len(modules)} modules, {len(ids.names)} ids, {len(VERBATIM_SEEN)} verbatim methods", file=sys.stderr)


if __name__ == "__main__":
    main()
