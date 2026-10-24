# Rusticc

Rusticc is COSMOS (OpenCentauri's firmware for the Elegoo Centauri Carbon) with Kalico replaced by Kaliru, the Rust port of Kalico.
Its logo is `Rusticc.png`, which replaces the OpenCentauri/COSMOS logo.

## How the code is organised

- This is kaliru branch `rusticc`, checked out at `D:\RustyKlipper\Rusticc`. kaliru `main` stays identical to Kalico.
- Each COSMOS change to Kalico is one commit on this branch, named after the COSMOS patch. The branch rebases onto kaliru `main`.
- The reference for parity is Kalico 5b59427 with COSMOS's patches applied. Build it with `rusticc/tools/make_ref.sh DEST`.
- `rusticc/docs/cosmos_audit.md` covers every COSMOS change: what it does, why COSMOS made it, and what Rusticc does about it.
- `rusticc/patches/` holds the COSMOS patches that had to be rebased onto 5b59427.
