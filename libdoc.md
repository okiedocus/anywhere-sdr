# anywhere-sdr Library Documentation

> **Legal Disclaimer**: GPS signal simulation may be regulated or restricted in your jurisdiction. This documentation is provided for educational and research purposes. Always comply with local laws and regulations before transmitting any RF signals.

---

## Table of Contents

1. [Overview](#overview)
2. [Adding the Crates to Your Project](#adding-the-crates-to-your-project)
3. [The `gps` Crate — Signal Generator Library](#the-gps-crate--signal-generator-library)
   - [Core Types](#core-types)
   - [Building a Signal Generator](#building-a-signal-generator)
   - [Builder Reference](#builder-reference)
   - [Running a Simulation to File](#running-a-simulation-to-file)
4. [The `libhackrf` Crate — HackRF Device Library](#the-libhackrf-crate--hackrf-device-library)
   - [Device Discovery and Connection](#device-discovery-and-connection)
   - [Device Configuration Reference](#device-configuration-reference)
   - [Transmit Mode](#transmit-mode)
5. [Direct Sample Access API](#direct-sample-access-api)
   - [Current State of the API](#current-state-of-the-api)
   - [Accessing Generated Samples via the Writer Buffer](#accessing-generated-samples-via-the-writer-buffer)
6. [Live HackRF GPS Signal Streaming](#live-hackrf-gps-signal-streaming)
   - [Architecture of a Live Stream](#architecture-of-a-live-stream)
   - [Step 1 — Generate Samples to a File](#step-1--generate-samples-to-a-file)
   - [Step 2 — Stream the File to HackRF](#step-2--stream-the-file-to-hackrf)
   - [Full End-to-End Example](#full-end-to-end-example)
7. [GPS L1 Signal Parameters for HackRF](#gps-l1-signal-parameters-for-hackrf)
8. [Error Handling](#error-handling)
9. [Important Constants](#important-constants)

---

## Overview

The workspace exposes two primary library crates for programmatic use:

| Crate | Purpose |
|---|---|
| `gps` | Generates GPS L1 C/A baseband I/Q samples from RINEX ephemeris data and a receiver position |
| `libhackrf` | Controls a HackRF One SDR device over USB — configures frequency, gain, sample rate, and drives bulk data transfer |

The typical live-streaming pipeline is:

```
RINEX file + position
        │
        ▼
  SignalGeneratorBuilder
        │  .build()
        ▼
  SignalGenerator::initialize()
        │  .run_simulation()
        ▼
  8-bit I/Q binary file
        │
        ▼
  HackRF::new_auto()  ──── configure freq / sample rate / gain
        │  .enter_tx_mode()
        ▼
  tx_queue().send() ──── bulk USB OUT transfers at 2.6 MS/s
        │
        ▼
  RF antenna at 1575.42 MHz (GPS L1)
```

---

## Adding the Crates to Your Project

Since these crates are not published to crates.io, reference them via path or git in your `Cargo.toml`:

```toml
[dependencies]
gps      = { path = "../anywhere-sdr/crates/gps" }
libhackrf = { path = "../anywhere-sdr/crates/libhackrf" }
```

Or from a git repository:

```toml
[dependencies]
gps       = { git = "https://github.com/lll9p/anywhere-sdr", package = "gps" }
libhackrf = { git = "https://github.com/lll9p/anywhere-sdr", package = "libhackrf" }
```

---

## The `gps` Crate — Signal Generator Library

### Core Types

```rust
use gps::{SignalGeneratorBuilder, SignalGenerator, MotionMode, DataFormat, Error};
```

| Type | Description |
|---|---|
| `SignalGeneratorBuilder` | Fluent builder for configuring simulation parameters |
| `SignalGenerator` | The simulation engine; holds all channel and ephemeris state |
| `MotionMode` | `Static` (fixed point) or `Dynamic` (trajectory from file) |
| `DataFormat` | `Bits1`, `Bits8`, or `Bits16` — bit depth of output I/Q samples |
| `Error` | Unified error type (implements `std::error::Error`) |

### Building a Signal Generator

All configuration goes through `SignalGeneratorBuilder`. The builder uses a fluent method-chaining API. Methods that can fail (parse errors, file I/O, validation) return `Result<Self, Error>`; methods that are infallible return `Self`.

```rust
use std::path::PathBuf;
use gps::SignalGeneratorBuilder;

let mut generator = SignalGeneratorBuilder::default()
    // Required: RINEX navigation file
    .navigation_file(Some(PathBuf::from("resources/brdc0010.22n")))?

    // Position — choose exactly one of these four:
    .location(Some(vec![35.6813, 139.7662, 10.0]))?   // lat(°), lon(°), alt(m)
    // .location_ecef(Some(vec![-3813477.0, 3554276.0, 3662785.0]))?  // ECEF m
    // .user_motion_file(Some(PathBuf::from("motion.csv")))?           // ECEF CSV
    // .user_motion_llh_file(Some(PathBuf::from("motion_llh.csv")))?   // LLH CSV
    // .user_motion_nmea_gga_file(Some(PathBuf::from("nmea.txt")))?    // NMEA GGA

    // Duration
    .duration(Some(60.0))                              // seconds

    // Output format — use Bits8 for HackRF
    .data_format(Some(8))?                             // 1, 8, or 16

    // Output file
    .output_file(Some(PathBuf::from("output/gps.bin")))

    // Optional settings
    .frequency(Some(2_600_000))?                       // Hz, default 2.6 MHz
    .time(Some("2022/01/01,00:00:00".to_string()))?    // or "now"
    .time_override(Some(false))                        // -T flag
    .ionospheric_disable(Some(false))                  // -i flag
    .path_loss(Some(35))                               // fixed TX gain, disables path loss
    .verbose(Some(true))
    .build()?;

generator.initialize()?;
generator.run_simulation()?;
```

### Builder Reference

#### `.navigation_file(Some(PathBuf))` → `Result<Self, Error>`

Loads a RINEX navigation file (`.nav`, `.yyN`). This is **required**. The file is parsed immediately at build time — ephemeris sets are organised into hourly buckets indexed `[time_set][prn-1]`.

```rust
.navigation_file(Some(PathBuf::from("brdc0010.22n")))?
```

Errors: `Error::NoEphemeris` if the file contains no valid data.

---

#### `.location(Some(vec![lat, lon, alt]))` → `Result<Self, Error>`

Static mode. Degrees for lat/lon, metres for altitude. Internally converted to ECEF.

```rust
.location(Some(vec![35.681298, 139.766247, 10.0]))?
```

---

#### `.location_ecef(Some(vec![x, y, z]))` → `Result<Self, Error>`

Static mode. All values in metres (ECEF).

```rust
.location_ecef(Some(vec![-3813477.954, 3554276.552, 3662785.237]))?
```

---

#### `.user_motion_file(Some(PathBuf))` → `Result<Self, Error>`

Dynamic mode. CSV file with ECEF X,Y,Z columns, one row per 100 ms step.

---

#### `.user_motion_llh_file(Some(PathBuf))` → `Result<Self, Error>`

Dynamic mode. CSV file with lat(°), lon(°), height(m), one row per 100 ms step.

---

#### `.user_motion_nmea_gga_file(Some(PathBuf))` → `Result<Self, Error>`

Dynamic mode. Text file of NMEA GGA sentences. Parsed and converted to ECEF.

---

#### `.duration(Some(seconds))` → `Self`

Simulation length in seconds. For static mode this directly sets the number of steps. For dynamic mode the actual duration is `min(duration_count, positions.len())`.

---

#### `.data_format(Some(bits))` → `Result<Self, Error>`

Sets the I/Q sample bit depth. Accepts `1`, `8`, or `16`. **HackRF expects 8-bit signed I/Q.**

```rust
.data_format(Some(8))?
```

---

#### `.frequency(Some(hz))` → `Result<Self, Error>`

Baseband sampling frequency in Hz. Default: `2_600_000` (2.6 MHz). Minimum: `1_000_000`.

---

#### `.output_file(Some(PathBuf))` → `Self`

Path for the binary I/Q output file. The file is created on `initialize()`.

---

#### `.time(Some(string))` → `Result<Self, Error>`

Simulation start time. Accepts `"YYYY/MM/DD,hh:mm:ss"` or `"now"` (uses system clock).

```rust
.time(Some("2022/01/01,00:00:00".to_string()))?
.time(Some("now".to_string()))?
```

If not set, the start time defaults to the earliest ephemeris epoch in the RINEX file.

---

#### `.time_override(Some(bool))` → `Self`

When `true` (the `-T` flag), shifts all ephemeris TOC/TOE values so the simulation start time aligns with the RINEX data, regardless of the actual calendar date in the file. Useful for replaying old RINEX files at an arbitrary time.

---

#### `.ionospheric_disable(Some(bool))` → `Self`

When `true` (the `-i` flag), disables ionospheric delay modelling. Useful for spacecraft simulations above the ionosphere.

---

#### `.path_loss(Some(gain))` → `Self`

When set, replaces the distance-based path loss calculation with a fixed gain value (scaled integer). This holds signal power constant across all satellites, which is the `-p` flag behaviour.

---

#### `.leap(Some(vec![week, day, delta_t]))` → `Self`

Overrides leap second parameters: GPS week number, day of week (1–7), and delta leap seconds (±128). Maps to the `-L` flag.

---

#### `.verbose(Some(bool))` → `Self`

When `true`, prints channel status tables (PRN, azimuth, elevation, range, ionospheric delay) to stderr every 30 seconds during simulation.

---

#### `.sample_rate(Some(seconds))` → `Self`

Time step between simulation updates. Default `0.1` (10 Hz). Rarely needs changing.

---

#### `.build()` → `Result<SignalGenerator, Error>`

Validates all settings and constructs the `SignalGenerator`. Possible errors:

| Error | Cause |
|---|---|
| `navigation_not_set` | No `.navigation_file()` called |
| `data_format_not_set` | No `.data_format()` called |
| `invalid_start_time` | Start time outside ephemeris range |
| `no_current_ephemerides` | No ephemeris set covers the start time |
| `duplicate_position` | Multiple position methods called |
| `wrong_positions` | Empty position list |
| `invalid_duration` | Negative duration |
| `invalid_gps_week/day/delta_leap_second` | Bad `-L` parameters |

---

### Running a Simulation to File

```rust
let mut generator = builder.build()?;

// Sets up channels, allocates satellites, creates the output file
generator.initialize()?;

// Runs all steps, writing I/Q samples to the output file
generator.run_simulation()?;
```

`run_simulation()` prints progress to stderr as `"Time into run = X.X"` and reports processing time on completion. All simulation steps run at 10 Hz (100 ms per step), accumulating one block of `sample_frequency × 0.1` I/Q sample pairs per step.

---

## The `libhackrf` Crate — HackRF Device Library

```rust
use libhackrf::prelude::*;
// Brings in: HackRF, Error, DeviceMode, TransceiverMode, SyncMode, Request, and all constants
```

### Device Discovery and Connection

#### Open the first connected device (most common)

```rust
let mut sdr = HackRF::new_auto()?;
```

#### Open a specific device by serial number

```rust
let mut sdr = HackRF::new(&"0123456789abcdef0123456789abcdef")?;
```

#### List all connected devices

```rust
let devices = HackRF::list_devices()?;
for device in &devices {
    println!("Serial: {:?}", device.serial_number());
}
```

#### Read device information

```rust
let board_id = sdr.board_id()?;          // u8 board ID
let version  = sdr.version()?;           // firmware version string
let api_ver  = sdr.device_version();     // u16 USB device version
let ((part_a, part_b), serial) = sdr.part_id_serial_read()?;
```

> **Note:** `part_id_serial_read()` consumes `sdr`. Call it last or re-open the device afterwards.

---

### Device Configuration Reference

All configuration methods take `&mut self`.

#### Set the RF frequency

```rust
sdr.set_freq(1_575_420_000)?;  // 1575.42 MHz — GPS L1
```

The frequency is split internally into MHz and Hz parts and sent as an 8-byte little-endian USB control packet.

---

#### Set the sample rate

```rust
// Recommended: automatic mode (finds best integer frequency/divider pair)
sdr.set_sample_rate_auto(2.6e6)?;     // 2.6 MS/s

// Manual mode (explicit frequency and divider)
sdr.set_sample_rate_manual(2_600_000, 1)?;
```

`set_sample_rate_auto()` finds an integer multiplier that minimises rounding error, up to a maximum of 32 iterations. `set_sample_rate_manual()` also automatically sets the baseband filter bandwidth to ≤75% of the sample rate using the MAX2837 chip's available bandwidth steps:

```
1.75, 2.5, 3.5, 5.0, 5.5, 6.0, 7.0, 8.0, 9.0, 10.0, 12.0, 14.0, 15.0, 20.0, 24.0, 28.0 MHz
```

---

#### Set the baseband filter bandwidth (optional override)

If you need a specific bandwidth (call *after* `set_sample_rate_*`):

```rust
sdr.set_baseband_filter_bandwidth(2_500_000)?;  // 2.5 MHz for 2.6 MS/s GPS
```

---

#### Set transmit VGA gain

```rust
sdr.set_txvga_gain(20)?;  // 0–47 dB, controls TX output power
```

For GPS simulation, start low (e.g. 20 dB) and increase only if needed. Over-driving the output risks interference.

---

#### Set LNA and RX VGA gain (receive path, not needed for TX)

```rust
sdr.set_lna_gain(16)?;    // 0–40 dB, steps of 8 dB
sdr.set_vga_gain(20)?;    // 0–62 dB, steps of 2 dB
```

---

#### Enable the RF amplifier

```rust
sdr.set_amp_enable(true)?;   // pre-amp on TX and RX paths
```

The amplifier adds approximately 11 dB. For short-range GPS simulation, leave it disabled.

---

#### Enable antenna port power

```rust
sdr.set_antenna_enable(1)?;  // powers the SMA port (for active antennas)
```

---

#### Enable clock output

```rust
sdr.set_clkout_enable(true)?;  // requires firmware ≥ 0x0103
```

---

#### Hardware sync mode

```rust
sdr.set_hw_sync_mode(0)?;  // 0 = off, 1 = on
```

---

#### Reset the device

```rust
sdr.reset()?;  // consumes sdr; requires firmware ≥ 0x0102
```

---

### Transmit Mode

```rust
// 1. Configure
sdr.set_freq(1_575_420_000)?;
sdr.set_sample_rate_auto(2.6e6)?;
sdr.set_txvga_gain(20)?;
sdr.set_amp_enable(false)?;

// 2. Enter TX mode
sdr.enter_tx_mode()?;

// 3. Obtain the bulk OUT endpoint
let mut tx = sdr.tx_queue()?;

// 4. Write chunks of exactly HACKRF_TRANSFER_BUFFER_SIZE bytes
// HACKRF_TRANSFER_BUFFER_SIZE = 262144 bytes (256 KB)
let chunk: Vec<u8> = /* ... your 8-bit signed I/Q data ... */;
tx.send_bulk(chunk, std::time::Duration::from_secs(5)).wait()?;

// 5. Stop
sdr.stop_tx()?;
```

The `tx_queue()` returns a `nusb::Endpoint<Bulk, Out>`. Each `send_bulk()` call blocks synchronously via `.wait()`.

**Buffer alignment**: HackRF firmware expects chunks that are multiples of its USB packet size. Using `HACKRF_TRANSFER_BUFFER_SIZE` (262 144 bytes) per transfer is safe and matches the original `hackrf_transfer` library's default.

---

## Direct Sample Access — FIFO Streaming to HackRF

This section describes how to stream GPS I/Q samples directly to a HackRF device in real time, eliminating the intermediate file entirely.

### How it works

`SignalGenerator::run_simulation_with_callback` drives the full simulation loop and delivers each 100 ms block of 8-bit signed I/Q bytes to a closure you provide. The closure sends the block into a bounded `mpsc::sync_channel`, which a dedicated HackRF thread drains and transmits over USB.

```
Generator thread                  HackRF thread
─────────────────────────────     ─────────────────────────
run_simulation_with_callback()    loop {
  → generate 100 ms of samples        rx.recv() → Vec<u8>
  → convert i16 → i8 → u8            tx_endpoint.send_bulk()
  → on_samples(&block)           }
      → sync_channel.send()
```

The `sync_channel` capacity (number of pre-buffered blocks) decouples the generator from USB transfer latency. A capacity of **4–8 blocks** is a safe starting point.

---

### What changed in the library

Two private methods were added to `SignalGenerator` in
`crates/gps/src/generator/signal_generator.rs`:

| Method | Visibility | Purpose |
|---|---|---|
| `generate_samples_into(&mut self, buf: &mut [i16])` | private | Fills an external buffer with I/Q samples, avoiding the borrow conflict with `IQWriter` |
| `run_simulation_with_callback<F: FnMut(&[u8]) -> Result<(), Error>>` | **pub** | Full simulation loop; delivers 8-bit blocks to the callback |

No output file is needed when using the callback API — omit `.output_file()` from the builder.

---

### Block geometry

At the default 2.6 MS/s sample rate and 100 ms step:

```
samples_per_block  = floor(2_600_000 × 0.1) = 260_000
bytes_per_block    = 260_000 × 2 (I + Q)   = 520_000 bytes  (~508 KB)
HackRF buffer size = HACKRF_TRANSFER_BUFFER_SIZE             = 262_144 bytes (256 KB)
```

Each 100 ms block is therefore **about two HackRF USB transfer buffers**. The HackRF thread should split each received `Vec<u8>` into `HACKRF_TRANSFER_BUFFER_SIZE`-sized chunks before calling `send_bulk`.

---

### Full FIFO streaming example

```rust
use std::{
    path::PathBuf,
    sync::mpsc,
    thread,
    time::Duration,
};
use gps::SignalGeneratorBuilder;
use libhackrf::prelude::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // ── 1. Bounded channel: GPS generator (producer) → HackRF (consumer) ──
    //    Capacity = 8 blocks × ~520 KB ≈ 4 MB of lookahead buffering.
    let (tx, rx) = mpsc::sync_channel::<Vec<u8>>(8);

    // ── 2. HackRF consumer thread ──────────────────────────────────────────
    let hackrf_thread = thread::spawn(move || -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let mut sdr = HackRF::new_auto()?;

        sdr.set_freq(1_575_420_000)?;      // GPS L1: 1575.42 MHz
        sdr.set_sample_rate_auto(2.6e6)?;  // must match generator frequency
        sdr.set_txvga_gain(20)?;           // TX gain 0–47 dB; start low
        sdr.set_amp_enable(false)?;        // RF amp off for benchtop testing

        sdr.enter_tx_mode()?;
        let mut endpoint = sdr.tx_queue()?;

        // Each 100 ms block (~520 KB) is split into 256 KB USB transfers.
        let mut chunk = vec![0u8; HACKRF_TRANSFER_BUFFER_SIZE];

        while let Ok(block) = rx.recv() {
            let mut offset = 0;
            while offset < block.len() {
                let end = (offset + HACKRF_TRANSFER_BUFFER_SIZE).min(block.len());
                let n   = end - offset;

                chunk[..n].copy_from_slice(&block[offset..end]);
                if n < HACKRF_TRANSFER_BUFFER_SIZE {
                    // Zero-pad the final partial chunk.
                    chunk[n..].fill(0);
                }

                endpoint
                    .send_bulk(chunk.clone(), Duration::from_secs(5))
                    .wait()?;

                offset += n;
            }
        }

        sdr.stop_tx()?;
        Ok(())
    });

    // ── 3. GPS signal generator (producer) ────────────────────────────────
    //    No .output_file() — samples go straight to the channel.
    let mut gen = SignalGeneratorBuilder::default()
        .navigation_file(Some(PathBuf::from("resources/brdc0010.22n")))?
        .location(Some(vec![35.681298, 139.766247, 10.0]))?   // lat, lon, alt
        .duration(Some(60.0))
        .data_format(Some(8))?          // 8-bit — required for HackRF format
        .frequency(Some(2_600_000))?    // 2.6 MS/s
        .verbose(Some(false))
        .build()?;

    gen.initialize()?;

    gen.run_simulation_with_callback(|block| {
        // Clone the slice into a Vec and send it to the HackRF thread.
        // sync_channel.send() blocks when the channel is full,
        // providing natural back-pressure.
        tx.send(block.to_vec())
            .map_err(|_| gps::Error::msg("HackRF channel closed"))
    })?;

    // Drop tx so the HackRF thread's rx.recv() returns Err and it exits.
    drop(tx);

    // Wait for the HackRF thread to finish flushing all buffers.
    hackrf_thread
        .join()
        .expect("HackRF thread panicked")?;

    Ok(())
}
```

---

### Looping the signal indefinitely

To broadcast the same simulated position continuously, wrap the generator call in a loop. Because `run_simulation_with_callback` advances internal GPS time, re-initialize the generator before each pass to reset the clock:

```rust
loop {
    gen.initialize()?;   // resets channels and GPS clock to start time
    gen.run_simulation_with_callback(|block| {
        tx.send(block.to_vec())
            .map_err(|_| gps::Error::msg("HackRF channel closed"))
    })?;
}
```

---

### File-based fallback

If real-time generation is too slow for your hardware, generate the samples to a file first and then stream the file:

```rust
// Step 1: generate to file
let mut gen = SignalGeneratorBuilder::default()
    .navigation_file(Some(PathBuf::from("resources/brdc0010.22n")))?
    .location(Some(vec![35.681298, 139.766247, 10.0]))?
    .duration(Some(60.0))
    .data_format(Some(8))?
    .frequency(Some(2_600_000))?
    .output_file(Some(PathBuf::from("output/gps_l1.bin")))
    .build()?;
gen.initialize()?;
gen.run_simulation()?;

// Step 2: stream file → HackRF
use std::{fs::File, io::{BufReader, Read}};

let mut sdr = HackRF::new_auto()?;
sdr.set_freq(1_575_420_000)?;
sdr.set_sample_rate_auto(2.6e6)?;
sdr.set_txvga_gain(20)?;
sdr.enter_tx_mode()?;
let mut endpoint = sdr.tx_queue()?;

let mut reader = BufReader::new(File::open("output/gps_l1.bin")?);
let mut chunk  = vec![0u8; HACKRF_TRANSFER_BUFFER_SIZE];

loop {
    let n = reader.read(&mut chunk)?;
    if n == 0 { break; }
    if n < HACKRF_TRANSFER_BUFFER_SIZE { chunk[n..].fill(0); }
    endpoint.send_bulk(chunk.clone(), Duration::from_secs(5)).wait()?;
}
sdr.stop_tx()?;
```

---

## GPS L1 Signal Parameters for HackRF

| Parameter | Value | Notes |
|---|---|---|
| Center frequency | 1 575 420 000 Hz | GPS L1 C/A |
| Baseband sample rate | 2 600 000 S/s | Default; min 1 000 000 |
| I/Q format | 8-bit signed (`i8`) | HackRF native format |
| Interleaving | `[I, Q, I, Q, ...]` | Byte-interleaved |
| TX VGA gain | 20–40 dB | Start low; increase for range |
| RF amplifier | Off | Not needed for benchtop/shielded testing |
| Baseband filter | 2 500 000 Hz | Auto-set by `set_sample_rate_auto` |
| Buffer size (USB) | 262 144 bytes | `HACKRF_TRANSFER_BUFFER_SIZE` |
| Samples per buffer | 131 072 I/Q pairs | At 8-bit (1 byte each I and Q) |
| Duration for 1 GB file | ~1615 seconds | 5.2 MB/s data rate |

### Sample rate and file size

```
file_size_bytes = duration_seconds × sample_rate × 2
               = 60 × 2_600_000 × 2 = 312_000_000 bytes ≈ 298 MB  (60s)
```

### Looping for extended simulation

The generated file can be looped by seeking back to the start of the file reader. GPS receivers handle looped signals as long as the simulated duration is longer than the receiver's acquisition time (typically > 30 seconds):

```rust
use std::io::Seek;

loop {
    reader.seek(std::io::SeekFrom::Start(0))?;
    // ... re-stream the file
}
```

---

## Error Handling

Both crates use `thiserror`-derived error enums.

### `gps::Error`

```rust
match gen.initialize() {
    Ok(()) => {},
    Err(gps::Error::NoEphemeris)             => eprintln!("No valid ephemeris data"),
    Err(gps::Error::InvalidStartTime)        => eprintln!("Start time outside ephemeris range"),
    Err(gps::Error::NoCurrentEphemerides)    => eprintln!("No ephemeris covers start time"),
    Err(gps::Error::DuplicatePosition)       => eprintln!("Multiple position sources set"),
    Err(gps::Error::ParsingError(msg))       => eprintln!("Parse error: {msg}"),
    Err(e)                                   => eprintln!("Error: {e}"),
}
```

### `libhackrf::error::Error`

```rust
match HackRF::new_auto() {
    Ok(sdr) => { /* use sdr */ },
    Err(Error::InvalidDevice)             => eprintln!("No HackRF found"),
    Err(Error::InvalidSerialNumber(s))    => eprintln!("Serial not found: {s}"),
    Err(Error::VersionMismatch { device, minimal }) =>
        eprintln!("Firmware {device} < required {minimal}"),
    Err(Error::Usb(e))                    => eprintln!("USB error: {e}"),
    Err(Error::Transfer(e))               => eprintln!("Transfer error: {e}"),
    Err(Error::Argument)                  => eprintln!("Gain value out of range"),
    Err(e)                                => eprintln!("Error: {e}"),
}
```

---

## Important Constants

From `libhackrf::constants` (re-exported via `prelude::*`):

| Constant | Value | Description |
|---|---|---|
| `HACKRF_USB_VID` | `0x1D50` | USB vendor ID |
| `HACKRF_ONE_USB_PID` | `0x6089` | USB product ID for HackRF One |
| `HACKRF_RX_ENDPOINT_ADDRESS` | `0x81` | Bulk IN endpoint |
| `HACKRF_TX_ENDPOINT_ADDRESS` | `0x02` | Bulk OUT endpoint |
| `HACKRF_TRANSFER_BUFFER_SIZE` | `262144` | USB transfer chunk size (256 KB) |
| `HACKRF_DEVICE_BUFFER_SIZE` | `32768` | Device internal buffer (32 KB) |
| `MHZ` | `1_000_000` | Convenience multiplier |

From `gps` (via `constants` workspace crate, not re-exported publicly):

| Constant | Value | Description |
|---|---|---|
| GPS L1 frequency | `1_575_420_000` Hz | Set on HackRF with `set_freq()` |
| Default sample rate | `2_600_000` Hz | Minimum recommended for GPS L1 C/A |
| Max visible satellites | 12 channels | `MAX_CHAN` |
| Max GPS satellites | 32 PRNs | `MAX_SAT` |
| Ephemeris sets | 100 | `EPHEM_ARRAY_SIZE` |
