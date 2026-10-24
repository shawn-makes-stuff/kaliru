# Rusticc

Rusticc is COSMOS (OpenCentauri's firmware for the Elegoo Centauri Carbon) with Kalico replaced by Kaliru, the Rust port of Kalico.
Its logo is `Rusticc.png`, which replaces the OpenCentauri/COSMOS logo.

## How the code is organised

- This is kaliru branch `rusticc`, checked out at `D:\RustyKlipper\Rusticc`. kaliru `main` stays identical to Kalico.
- Each COSMOS change to Kalico is one commit on this branch, named after the COSMOS patch. The branch rebases onto kaliru `main`.
- The reference for parity is Kalico 5b59427 with COSMOS's patches applied. Build it with `rusticc/tools/make_ref.sh DEST`.
- `rusticc/docs/cosmos_audit.md` covers every COSMOS change: what it does, why COSMOS made it, and what Rusticc does about it.
- `rusticc/patches/` holds the COSMOS patches that had to be rebased onto 5b59427.

## Status (2026-10-24)

Ported and checked against the COSMOS reference (`rusticc/tools/ref_setup.sh`, then `rusticc/tools/case_cmp.sh`):
the host patches 0001-remove-save-config-subfile-check, 0002-reduce-calibration-difference-tolerance,
0001-Reduce-log-rotate-threshold, 0001-implement-pulldown-resistor-in-thermistor-calc, 0001-probe-home-z-includes-z-offset,
0001-force-specific-input-shaper, 0004-allow-config-exclude, and the hx711s/hx717s load cell (0007). The full case set
is 159/160 identical (printers/13 is a known timing case).

`rusticc/tools/cc1_cmp.sh` runs the user's CC1 config (AFC, AFC_ACE, knock) in batch mode with the printer's own MCU
dictionaries (`rusticc/dicts`). Startup is identical to COSMOS.

Not ported yet: drv8833, canvas_rfid, canvas_filament_sensor (not in the user's CC1 config), and the firmware patches.
