#!/usr/bin/env python3
# Report of a KALIRU_PCPROF file (src/pcprof.rs): samples per function and
# per caller, mapped with addr2line on an unstripped build of the same code.
# usage: pcprof_report.py PROFILE UNSTRIPPED_BINARY [ADDR2LINE] [TID]
import collections
import subprocess
import sys

prof, binary = sys.argv[1], sys.argv[2]
addr2line = sys.argv[3] if len(sys.argv) > 3 else "addr2line"
only_tid = sys.argv[4] if len(sys.argv) > 4 else None
maps, rows, total = [], [], 0
for line in open(prof):
    f = line.split()
    if f[0] == "map":
        lo, hi = (int(x, 16) for x in f[1].split("-"))
        maps.append((lo, hi, int(f[3], 16)))
    elif f[0] == "samples":
        total = int(f[1])
    else:
        rows.append((f[0], int(f[1], 16), int(f[2], 16), int(f[3])))


def vaddr(a):
    # PIE: the binary's own mappings; file offset == virtual address
    for lo, hi, off in maps:
        if lo <= a < hi:
            return a - lo + off
    return None


addrs = sorted({v for _, pc, lr, _ in rows for v in (vaddr(pc), vaddr(lr)) if v is not None})
out = subprocess.run([addr2line, "-f", "-C", "-e", binary] + ["%x" % a for a in addrs],
                     capture_output=True, text=True).stdout.split("\n")
name = {a: out[2 * i] for i, a in enumerate(addrs)}
fn = collections.Counter()
pair = collections.Counter()
tids = collections.Counter()
for tid, pc, lr, c in rows:
    tids[tid] += c
    if only_tid and tid != only_tid:
        continue
    f = name.get(vaddr(pc), "<outside binary: libc / libpython / ...>")
    fn[f] += c
    pair[(f, name.get(vaddr(lr), "?"))] += c
n = sum(fn.values())
print("samples %d; per thread: %s" % (total, ", ".join("%s=%d" % t for t in tids.most_common())))
print("\n== functions (%d samples)" % n)
for f, c in fn.most_common(40):
    print("%5.1f%% %6d  %s" % (100.0 * c / n, c, f[:150]))
print("\n== function <- caller (lr)")
for (f, l), c in pair.most_common(40):
    print("%5.1f%% %6d  %s  <-  %s" % (100.0 * c / n, c, f[:90], l[:90]))
