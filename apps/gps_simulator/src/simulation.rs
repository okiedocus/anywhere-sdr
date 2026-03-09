use std::{
    path::PathBuf,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    thread,
    time::Duration,
};

use gps::SignalGeneratorBuilder;
use libhackrf::prelude::*;

// GPS L1 C/A centre frequency
const GPS_L1_HZ: u64 = 1_575_420_000;

/// Settings passed from the UI to the simulation thread.
#[derive(Clone)]
pub struct SimSettings {
    /// Baseband sampling frequency in Hz (must be ≥ 1 000 000).
    pub frequency: usize,
    /// HackRF TX VGA gain in dB (0–47).
    pub txvga_gain: u16,
    /// Whether to enable the HackRF RF pre-amplifier.
    pub amp_enable: bool,
}

/// Shared simulation state; updated by the worker thread and read by the UI.
#[derive(Default, Clone)]
pub struct SimState {
    pub status: SimStatus,
    /// Simulation step index (each step = 100 ms).
    pub current_step: usize,
    /// Total number of steps derived from the motion file length.
    pub total_steps: usize,
    /// Raw bytes delivered to HackRF so far.
    pub bytes_sent: u64,
    /// Human-readable error description, set when `status == Error`.
    pub error: Option<String>,
}

#[derive(Default, Clone, PartialEq)]
pub enum SimStatus {
    #[default]
    Idle,
    Running,
    Done,
    /// Stopped cleanly by the user.
    Stopped,
    Error,
}

/// Entry point called from the UI after spawning a dedicated thread.
pub fn run(
    rinex_path: PathBuf,
    motion_path: PathBuf,
    settings: SimSettings,
    state: Arc<Mutex<SimState>>,
    stop: Arc<AtomicBool>,
) {
    match do_run(&rinex_path, &motion_path, &settings, Arc::clone(&state), Arc::clone(&stop)) {
        Ok(()) => {
            let mut s = state.lock().unwrap();
            if s.status == SimStatus::Running {
                s.status = SimStatus::Done;
            }
        }
        Err(e) => {
            let mut s = state.lock().unwrap();
            // If the user pressed Stop, report it as Stopped rather than Error.
            if stop.load(Ordering::Relaxed) {
                s.status = SimStatus::Stopped;
            } else if s.status == SimStatus::Running {
                s.status = SimStatus::Error;
                if s.error.is_none() {
                    s.error = Some(e.to_string());
                }
            }
        }
    }
}

fn do_run(
    rinex_path: &PathBuf,
    motion_path: &PathBuf,
    settings: &SimSettings,
    state: Arc<Mutex<SimState>>,
    stop: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // ── 1. Build and initialise the signal generator ──────────────────────
    let mut generator = SignalGeneratorBuilder::default()
        .navigation_file(Some(rinex_path.clone()))?
        .user_motion_file(Some(motion_path.clone()))?
        .data_format(Some(8))? // 8-bit signed I/Q — HackRF native
        .frequency(Some(settings.frequency))?
        .verbose(Some(false))
        .build()?;

    generator.initialize()?;

    // Expose total step count to the UI.
    state.lock().unwrap().total_steps = generator.simulation_step_count;

    // ── 2. Open and configure HackRF ──────────────────────────────────────
    let mut sdr = HackRF::new_auto()?;
    sdr.set_freq(GPS_L1_HZ)?;
    sdr.set_sample_rate_auto(settings.frequency as f64)?;
    sdr.set_txvga_gain(settings.txvga_gain)?;
    sdr.set_amp_enable(settings.amp_enable)?;
    sdr.enter_tx_mode()?;

    // ── 3. FIFO channel: generator (producer) → HackRF thread (consumer) ─
    //    Capacity of 8 blocks ≈ 8 × 520 KB ≈ 4 MB of lookahead.
    let (tx, rx) = mpsc::sync_channel::<Vec<u8>>(8);
    // Clone the sender for the callback closure; the original `tx` is kept
    // alive here so we can drop it explicitly after the simulation finishes,
    // signalling the HackRF thread to flush and exit.
    let tx_cb = tx.clone();

    // Shared slot for errors reported by the HackRF thread.
    let hackrf_err: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
    let hackrf_err_t = Arc::clone(&hackrf_err);
    let state_t = Arc::clone(&state);

    // ── 4. HackRF consumer thread ──────────────────────────────────────────
    let hackrf_thread = thread::spawn(move || {
        let mut endpoint = match sdr.tx_queue() {
            Ok(ep) => ep,
            Err(e) => {
                *hackrf_err_t.lock().unwrap() = Some(e.to_string());
                return;
            }
        };

        // Each 100 ms block (~520 KB at 2.6 MS/s) must be split into
        // HACKRF_TRANSFER_BUFFER_SIZE (256 KB) chunks for the USB DMA engine.
        let mut chunk = vec![0u8; HACKRF_TRANSFER_BUFFER_SIZE];

        while let Ok(block) = rx.recv() {
            for window in block.chunks(HACKRF_TRANSFER_BUFFER_SIZE) {
                let n = window.len();
                chunk[..n].copy_from_slice(window);
                // Zero-pad the final partial chunk.
                if n < HACKRF_TRANSFER_BUFFER_SIZE {
                    chunk[n..].fill(0);
                }

                if let Err(e) = endpoint
                    .transfer_blocking(chunk.clone().into(), Duration::from_secs(5))
                    .into_result()
                {
                    *hackrf_err_t.lock().unwrap() = Some(e.to_string());
                    return;
                }

                state_t.lock().unwrap().bytes_sent += n as u64;
            }
        }

        // Channel was closed — flush and stop TX.
        sdr.stop_tx().ok();
    });

    // ── 5. GPS simulation loop with streaming callback ─────────────────────
    let mut step: usize = 0;
    let state_cb = Arc::clone(&state);
    let stop_cb = Arc::clone(&stop);

    let sim_result = generator.run_simulation_with_callback(move |block| {
        // Honour stop requests by aborting with an error (the caller
        // interprets this case as SimStatus::Stopped, not Error).
        if stop_cb.load(Ordering::Relaxed) {
            return Err(gps::Error::msg("stopped"));
        }

        // Forward the block to the HackRF thread.
        tx_cb
            .send(block.to_vec())
            .map_err(|_| gps::Error::msg("HackRF channel closed"))?;

        step += 1;
        state_cb.lock().unwrap().current_step = step;
        Ok(())
    });

    // Drop the original sender so the HackRF thread's rx.recv() returns Err
    // and it exits its loop.
    drop(tx);

    // Wait for the HackRF thread to finish flushing.
    hackrf_thread.join().ok();

    // Surface any error the HackRF thread recorded.
    if let Some(err) = hackrf_err.lock().unwrap().take() {
        let mut s = state.lock().unwrap();
        s.error = Some(format!("HackRF error: {err}"));
        return Err(err.into());
    }

    // Propagate any simulation error (e.g. "stopped" or file I/O issues).
    sim_result?;

    Ok(())
}
