mod app;
mod simulation;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([580.0, 520.0])
            .with_min_inner_size([480.0, 420.0])
            .with_title("GPS L1 C/A Simulator"),
        ..Default::default()
    };

    eframe::run_native(
        "GPS L1 C/A Simulator",
        options,
        Box::new(|_cc| Ok(Box::new(app::App::default()))),
    )
}
