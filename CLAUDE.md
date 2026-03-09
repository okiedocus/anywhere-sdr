# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
# Build
cargo build --release

# Check for errors
cargo check --all

# Lint (warnings are errors)
cargo clippy --all -- -D warnings

# Format (requires nightly due to unstable rustfmt options)
cargo +nightly fmt --all

# Run standard tests
cargo test

# Run integration tests (release mode required — compares output against reference C implementation)
cargo test --release

# Run a specific integration test
cargo test --release -p gps --test test-generator test_data_format_1bit

# Run hardware-dependent tests (requires physical HackRF device)
cargo test -p libhackrf -- --ignored
```

Before running integration tests locally, create the output directory:
```bash
mkdir -p output
```

## Architecture

This is a Cargo workspace with a binary app and several library crates.

### Crates

- **`apps/gpssim`** — CLI binary. Parses args via `clap` and delegates to `gps::SignalGeneratorBuilder`.
- **`crates/gps`** — Core library. Contains the signal generation pipeline. Public API is `SignalGeneratorBuilder` → `SignalGenerator`. Internal modules: `channel`, `datetime`, `delay`, `ephemeris`, `generator`, `io`, `ionoutc`, `propagation`, `table`.
- **`crates/rinex`** — RINEX navigation file parser using `pest` grammar (`rinex.pest`). Extracts ephemeris, ionospheric, and UTC parameters.
- **`crates/parsing`** — Input format parsers: ECEF user motion CSV, LLH user motion CSV, NMEA GGA sentences.
- **`crates/geometry`** — Coordinate types (`Ecef`, `Location`/LLH) and transformations between them.
- **`crates/constants`** — Shared GPS constants (`MAX_SAT`, `MAX_CHAN`, `EPHEM_ARRAY_SIZE`, etc.).
- **`crates/libhackrf`** — HackRF SDR hardware bindings using `nusb` (replaces `rusb` from upstream `libhackrf-rs`).

### Signal Generation Flow

1. `SignalGeneratorBuilder` loads RINEX nav data (→ ephemeris array indexed `[time_set][sat_prn]`), converts position input to ECEF, and selects the valid ephemeris set for the simulation start time.
2. `SignalGenerator::initialize()` sets up satellite channels.
3. `SignalGenerator::run_simulation()` iterates simulation steps at 10 Hz (default), accumulating I/Q samples per step and writing them to the output file in 1/8/16-bit format via `IQWriter`.
4. Alternatively, `SignalGenerator::run_simulation_with_callback<F: FnMut(&[u8]) -> Result<()>>` drives the same loop but delivers each 100 ms block of 8-bit signed I/Q bytes to a closure instead of a file — no output file needed. Intended for live HackRF streaming via a producer/consumer channel.

### Key Design Notes

- Integration tests in `crates/gps/tests/test-generator.rs` compile and run the reference C implementation (`resources/gpssim.c`) and byte-compare its output against the Rust implementation. This is why they only run in `--release` mode.
- `CARGO_WORKSPACE_DIR` is set via `.cargo/config.toml` so tests can locate `resources/` files regardless of where `cargo test` is invoked.
- Clippy is configured at workspace level with `pedantic = deny` and `style = deny`. GPS signal processing numeric casts (`cast_possible_truncation`, `cast_sign_loss`, `cast_precision_loss`) are explicitly allowed.
- Commits must follow Conventional Commits format (enforced by pre-commit hook).
