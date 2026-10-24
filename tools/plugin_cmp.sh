# Python plugin parity: runs tests/cases/plugins/*.test (or the given ones)
# with Kalico and with kaliru, and diffs them like case_cmp.sh.
#  - test plugins tests/plugins/*.py go into the reference's klippy/plugins
#    (git-ignored there) for the run
#  - AFC (Originals_Base/addons/afc_extras, kept outside this repo) goes into
#    klippy/extras and AFC_ACE into klippy/plugins for the run
#  - plugin state files (AFC's VarFile) live in /tmp/kaliru_plugin_state,
#    emptied before each side runs
# kaliru is pointed at the same klippy directory. Needs CPython 3.15+ with a
# shared libpython: KALIRU_LIBPYTHON (default: the uv build), and the
# reference venv ~/rk/venv315 (uv venv -p 3.15 + scripts/klippy-requirements.txt).
# usage: plugin_cmp.sh [test files...]
cd "$(dirname "$0")/.."
K=~/rk/kalico/klippy
ADDONS=${ADDONS:-$(realpath ../Originals_Base/addons)}
installed=()
for f in tests/plugins/*.py; do cp "$f" $K/plugins/; installed+=("$K/plugins/$(basename "$f")"); done
for f in "$ADDONS"/afc_extras/*.py; do cp "$f" $K/extras/; installed+=("$K/extras/$(basename "$f")"); done
cp -r "$ADDONS/plugins/AFC_ACE" $K/plugins/ && installed+=("$K/plugins/AFC_ACE")
trap 'rm -rf "${installed[@]}" /tmp/kaliru_plugin_state' EXIT
export KALIRU_KLIPPY_DIR=$K
export KALIRU_LIBPYTHON=${KALIRU_LIBPYTHON:-$HOME/.local/share/uv/python/cpython-3.15-linux-x86_64-gnu/lib/libpython3.15.so.1.0}
# both sides on the same CPython 3.15 and packages (plugins log tracebacks
# with stdlib paths): the reference in ~/rk/venv315 (tools/setup_env.sh),
# kaliru's embedded python with that venv's site-packages
REF_KLIPPY=${REF_KLIPPY:-$HOME/rk/venv315/bin/python -m klippy}
KALIRU_PYTHONPATH=$HOME/rk/venv315/lib/python3.15/site-packages
[ $# -eq 0 ] && set -- tests/cases/plugins/*.test
tests=()
for t in "$@"; do tests+=("$(realpath "$t")"); done
fresh() { rm -rf /tmp/kaliru_plugin_state; mkdir -p /tmp/kaliru_plugin_state; }
rm -rf ~/rk/case_py ~/rk/case_rs
fresh
python3 tools/refrun.py --out ~/rk/case_py ${REF_KLIPPY:+--klippy "$REF_KLIPPY"} -j 1 "${tests[@]}" < /dev/null | tail -1
fresh
PYTHONPATH=$KALIRU_PYTHONPATH python3 tools/refrun.py --out ~/rk/case_rs --klippy "${KALIRU_BIN:-$HOME/rk/target/release/kaliru}" -j 1 "${tests[@]}" < /dev/null | tail -1
python3 tools/refdiff.py ~/rk/case_py ~/rk/case_rs
