cd /mnt/d/RustyKlipper/Rusticc && . ~/.cargo/env && CARGO_PROFILE_RELEASE_STRIP=none CARGO_PROFILE_RELEASE_DEBUG=1 CARGO_TARGET_DIR=~/rk/target-rusticc-dbg cargo build --release 2>&1 | grep -E "^error" -A10
RUST_BACKTRACE=1 KALIRU_BIN=~/rk/target-rusticc-dbg/release/kaliru bash rusticc/tools/cc1_cmp.sh rusticc/cases/cc1_restart.gcode > /dev/null 2>&1
grep -n "panicked" -A40 ~/rk/cc1_rs/cc1/0/log | grep "kaliru::\|at ./src\|at src" | head -30
