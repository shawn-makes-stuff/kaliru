#!/usr/bin/env python3
# Report of a KALIRU_PCPROF file (src/pcprof.rs): samples per function and
# per caller. Addresses in the kaliru binary are mapped with addr2line on an
# unstripped build of the same code; addresses in shared libraries with the
# nearest dynamic symbol of a local copy of that library (LIBDIR/<basename>).
# usage: pcprof_report.py PROFILE UNSTRIPPED_BINARY ADDR2LINE NM LIBDIR [TID]
import bisect
import collections
import os
import subprocess
import sys

prof, binary, addr2line, nm, libdir = sys.argv[1:6]
only_tid = sys.argv[6] if len(sys.argv) > 6 else None
maps, rows, total = [], [], 0
for line in open(prof):
    f = line.split()
    if f[0] == "map":
        lo, hi = (int(x, 16) for x in f[1].split("-"))
        maps.append((lo, hi, int(f[3], 16), f[-1]))
    elif f[0] == "samples":
        total = int(f[1])
    else:
        rows.append((f[0], int(f[1], 16), int(f[2], 16), int(f[3])))
exe = next((m[3] for m in maps if ".so" not in os.path.basename(m[3])), "")


def locate(a):
    """(file, file offset) of an address"""
    for lo, hi, off, path in maps:
        if lo <= a < hi:
            return path, a - lo + off
    return None, a


syms = {}


def libsym(path, off):
    if path not in syms:
        local = os.path.join(libdir, os.path.basename(path))
        table = []
        if os.path.exists(local):
            out = subprocess.run([nm, "-D", "--defined-only", local], capture_output=True, text=True).stdout
            for l in out.split("\n"):
                p = l.split()
                if len(p) == 3 and p[1] in "TtWw":
                    table.append((int(p[0], 16), p[2]))
        table.sort()
        syms[path] = table
    table = syms[path]
    i = bisect.bisect_right(table, (off, "￿")) - 1
    base = os.path.basename(path)
    return "%s:%s" % (base, table[i][1] if i >= 0 else hex(off))


addrs = sorted({o for _, pc, lr, _ in rows for p, o in (locate(pc), locate(lr)) if p == exe})
out = subprocess.run([addr2line, "-f", "-C", "-e", binary] + ["%x" % a for a in addrs],
                     capture_output=True, text=True).stdout.split("\n")
binname = {a: out[2 * i] for i, a in enumerate(addrs)}


def name(a):
    path, off = locate(a)
    if path is None:
        return "?"
    if path == exe:
        return binname.get(off, "?")
    return libsym(path, off)


fn = collections.Counter()
pair = collections.Counter()
tids = collections.Counter()
for tid, pc, lr, c in rows:
    tids[tid] += c
    if only_tid and tid != only_tid:
        continue
    fn[name(pc)] += c
    pair[(name(pc), name(lr))] += c
n = sum(fn.values())
print("samples %d; per thread: %s" % (total, ", ".join("%s=%d" % t for t in tids.most_common())))
print("\n== functions (%d samples)" % n)
for f, c in fn.most_common(45):
    print("%5.1f%% %6d  %s" % (100.0 * c / n, c, f[:150]))
print("\n== function <- caller (lr)")
for (f, l), c in pair.most_common(45):
    print("%5.1f%% %6d  %s  <-  %s" % (100.0 * c / n, c, f[:90], l[:90]))
