#!/bin/sh
# Runs Rusticc on the CC1 in place of COSMOS's klippy, with the same
# arguments and environment as /etc/init.d/klipper. COSMOS's klippy must be
# stopped first (/etc/init.d/klipper stop); the way back is
# /etc/init.d/klipper start. Rusticc lives in /user-resource/rusticc (the
# root filesystem is read-only); the python plugins (AFC in klippy/extras,
# /etc/klipper/plugins) are read from COSMOS's klippy directory.
export TMPDIR="/user-resource/.tmp"
export KALIRU_KLIPPY_DIR="/usr/share/klipper/klippy"
export KALIRU_LIBPYTHON="/usr/lib/libpython3.15.so.1.0"
exec /user-resource/rusticc/rusticc /etc/klipper/config/printer.cfg \
    -l /board-resource/klippy.log -a /run/klippy.sock "$@"
