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

use eframe::egui;
use egui::{Align, Color32, Layout, RichText};

use crate::simulation::{SimSettings, SimState, SimStatus};

// ── File-dialog helper ────────────────────────────────────────────────────────

/// Opens a native file-picker dialog in a background thread (so the UI stays
/// responsive).  Returns a `Receiver` that yields `Some(path)` when the user
/// picks a file, or `None` if they cancel.
fn open_file_dialog(
    title: impl Into<String>,
    filters: &[(&'static str, &'static [&'static str])],
) -> mpsc::Receiver<Option<PathBuf>> {
    let (tx, rx) = mpsc::channel();
    let title = title.into();
    // Collect the filter data before moving into the thread.
    let filters: Vec<(&'static str, &'static [&'static str])> = filters.to_vec();

    thread::spawn(move || {
        let mut dialog = rfd::FileDialog::new().set_title(&title);
        for (name, exts) in filters {
            dialog = dialog.add_filter(name, exts);
        }
        tx.send(dialog.pick_file()).ok();
    });

    rx
}

// ── App state ─────────────────────────────────────────────────────────────────

pub struct App {
    // ── selected files ────────────────────────────────────────────────────
    rinex_path: Option<PathBuf>,
    motion_path: Option<PathBuf>,

    // pending file-dialog results (None when no dialog is open)
    rinex_dialog: Option<mpsc::Receiver<Option<PathBuf>>>,
    motion_dialog: Option<mpsc::Receiver<Option<PathBuf>>>,

    // ── HackRF settings ───────────────────────────────────────────────────
    txvga_gain: u16,
    amp_enable: bool,
    frequency: usize,

    // ── simulation state ──────────────────────────────────────────────────
    sim_state: Arc<Mutex<SimState>>,
    stop_flag: Arc<AtomicBool>,
    sim_thread: Option<thread::JoinHandle<()>>,
}

impl Default for App {
    fn default() -> Self {
        Self {
            rinex_path: None,
            motion_path: None,
            rinex_dialog: None,
            motion_dialog: None,
            txvga_gain: 20,
            amp_enable: false,
            frequency: 2_600_000,
            sim_state: Arc::new(Mutex::new(SimState::default())),
            stop_flag: Arc::new(AtomicBool::new(false)),
            sim_thread: None,
        }
    }
}

// ── eframe::App implementation ────────────────────────────────────────────────

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // ── poll pending file-dialog results ──────────────────────────────
        if let Some(rx) = &self.rinex_dialog {
            if let Ok(path) = rx.try_recv() {
                self.rinex_path = path;
                self.rinex_dialog = None;
            }
        }
        if let Some(rx) = &self.motion_dialog {
            if let Ok(path) = rx.try_recv() {
                self.motion_path = path;
                self.motion_dialog = None;
            }
        }

        // ── clean up finished simulation thread ───────────────────────────
        if self
            .sim_thread
            .as_ref()
            .map(|h| h.is_finished())
            .unwrap_or(false)
        {
            if let Some(h) = self.sim_thread.take() {
                h.join().ok();
            }
        }

        // ── keep repainting while the simulation is running ───────────────
        if self.sim_state.lock().unwrap().status == SimStatus::Running {
            ctx.request_repaint_after(Duration::from_millis(150));
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            self.draw_ui(ui);
        });
    }
}

// ── UI drawing ────────────────────────────────────────────────────────────────

impl App {
    fn draw_ui(&mut self, ui: &mut egui::Ui) {
        ui.heading("GPS L1 C/A Simulator");
        ui.add_space(10.0);

        // ── Input files ───────────────────────────────────────────────────
        ui.group(|ui| {
            ui.label(RichText::new("Input Files").strong());
            ui.add_space(4.0);

            self.draw_file_row(
                ui,
                "RINEX Nav File",
                &self.rinex_path.clone(),
                self.rinex_dialog.is_some(),
                |app| {
                    app.rinex_dialog = Some(open_file_dialog(
                        "Select RINEX Navigation File",
                        &[("RINEX Navigation", &["nav", "n", "22n", "23n", "24n", "25n"])],
                    ));
                },
            );

            ui.add_space(2.0);

            self.draw_file_row(
                ui,
                "Motion CSV (ECEF)",
                &self.motion_path.clone(),
                self.motion_dialog.is_some(),
                |app| {
                    app.motion_dialog = Some(open_file_dialog(
                        "Select User Motion File (ECEF x,y,z CSV)",
                        &[("CSV files", &["csv"])],
                    ));
                },
            );
        });

        ui.add_space(8.0);

        // ── HackRF settings ───────────────────────────────────────────────
        let running = self.sim_thread.is_some();
        ui.add_enabled_ui(!running, |ui| {
            ui.group(|ui| {
                ui.label(RichText::new("HackRF Settings").strong());
                ui.add_space(4.0);

                ui.horizontal(|ui| {
                    ui.label("TX VGA Gain:");
                    ui.add(egui::Slider::new(&mut self.txvga_gain, 0..=47).suffix(" dB"));
                });
                ui.horizontal(|ui| {
                    ui.label("Sample Rate:");
                    ui.add(
                        egui::Slider::new(&mut self.frequency, 1_000_000..=20_000_000)
                            .suffix(" Hz")
                            .step_by(100_000.0),
                    );
                });
                ui.checkbox(&mut self.amp_enable, "Enable RF Amplifier");
                ui.label(
                    RichText::new("⚠ Transmitting GPS signals may be illegal. Use only in a shielded environment.")
                        .small()
                        .color(Color32::YELLOW),
                );
            });
        });

        ui.add_space(8.0);

        // ── Control buttons ───────────────────────────────────────────────
        let ready = self.rinex_path.is_some() && self.motion_path.is_some() && !running;

        ui.horizontal(|ui| {
            ui.add_enabled_ui(ready, |ui| {
                if ui
                    .button(RichText::new("  ▶  Start Simulation  ").size(15.0))
                    .clicked()
                {
                    self.start_simulation();
                }
            });

            if running
                && ui
                    .button(RichText::new("  ■  Stop  ").size(15.0))
                    .clicked()
            {
                self.stop_flag.store(true, Ordering::Relaxed);
            }
        });

        ui.add_space(8.0);

        // ── Status panel ──────────────────────────────────────────────────
        ui.group(|ui| {
            ui.label(RichText::new("Status").strong());
            ui.add_space(4.0);

            let state = self.sim_state.lock().unwrap().clone();

            let (status_text, status_color) = match &state.status {
                SimStatus::Idle => ("Idle", Color32::GRAY),
                SimStatus::Running => ("Running…", Color32::GREEN),
                SimStatus::Done => ("Done", Color32::LIGHT_BLUE),
                SimStatus::Stopped => ("Stopped by user", Color32::GOLD),
                SimStatus::Error => ("Error", Color32::RED),
            };
            ui.label(RichText::new(status_text).color(status_color));

            if let Some(err) = &state.error {
                ui.colored_label(Color32::RED, err);
            }

            // Progress bar
            let progress = if state.total_steps > 0 {
                state.current_step as f32 / state.total_steps as f32
            } else {
                0.0
            };
            ui.add(
                egui::ProgressBar::new(progress)
                    .text(format!(
                        "{:.0}%  ({:.1} s / {:.1} s)",
                        progress * 100.0,
                        state.current_step as f64 / 10.0,
                        state.total_steps as f64 / 10.0,
                    ))
                    .desired_width(f32::INFINITY),
            );

            ui.label(format!(
                "Bytes transmitted: {:.2} MB",
                state.bytes_sent as f64 / 1_000_000.0
            ));
        });
    }

    /// Draws a single file-selection row with a label, truncated path display,
    /// and a Browse button.
    fn draw_file_row(
        &mut self,
        ui: &mut egui::Ui,
        label: &str,
        current: &Option<PathBuf>,
        dialog_open: bool,
        on_browse: impl FnOnce(&mut App),
    ) {
        ui.horizontal(|ui| {
            ui.label(format!("{label}:"));

            // Show only the file name to keep the layout compact.
            let display = current
                .as_deref()
                .and_then(|p| p.file_name())
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_else(|| "None selected".into());

            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                let btn_text = if dialog_open { "…" } else { "Browse…" };
                if ui.add_enabled(!dialog_open, egui::Button::new(btn_text)).clicked() {
                    on_browse(self);
                }
                ui.label(RichText::new(display).monospace().weak());
            });
        });
    }

    /// Spawns the simulation worker thread.
    fn start_simulation(&mut self) {
        // Reset shared state.
        *self.sim_state.lock().unwrap() = SimState {
            status: SimStatus::Running,
            ..Default::default()
        };
        self.stop_flag.store(false, Ordering::Relaxed);

        let rinex_path = self.rinex_path.clone().expect("checked before calling");
        let motion_path = self.motion_path.clone().expect("checked before calling");
        let settings = SimSettings {
            frequency: self.frequency,
            txvga_gain: self.txvga_gain,
            amp_enable: self.amp_enable,
        };
        let state = Arc::clone(&self.sim_state);
        let stop = Arc::clone(&self.stop_flag);

        self.sim_thread = Some(thread::spawn(move || {
            crate::simulation::run(rinex_path, motion_path, settings, state, stop);
        }));
    }
}
