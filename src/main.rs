mod modules;

use eframe::egui;
use modules::serial_lib::{DEFAULT_BAUD, DEFAULT_PORT};
use modules::serial_worker::{
    GcodeSerialCountMode, SerialCommand, SerialEvent, SerialWorker, SessionConfig,
};
use rfd::FileDialog;
use std::fs;
use std::time::Duration;

const JOG_STEP_LUT: [f32; 4] = [0.01, 0.1, 1.0, 10.0];
const LEFT_PANEL_BUTTON_LABEL_WIDTH: usize = "Button 0".len();

fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("RustLaser GUI v0.2")
            .with_inner_size([1280.0, 800.0])
            .with_min_inner_size([1024.0, 700.0]),
        ..Default::default()
    };

    eframe::run_native(
        "RustLaser GUI",
        options,
        Box::new(|cc| Ok(Box::new(RustLaserApp::new(cc)))),
    )
}

struct RustLaserApp {
    file_path: String,
    gcode_lines: Vec<String>,
    gcode_polyline: Vec<[f32; 2]>,
    status: String,
    serial_address: String,
    serial_baud: u32,
    serial_worker: SerialWorker,
    serial_connected: bool,
    serial_input: String,
    serial_log: Vec<String>,
    jog_step_index: i32,
    jog_feed: f32,
    canvas_center: [f32; 2],
    canvas_zoom: f32,
    show_millimeter_grid: bool,
    show_workspace_grid: bool,
    show_workspace_axes: bool,
    session_config: SessionConfig,
    print_active: bool,
    gcode_progress_sent: usize,
    gcode_progress_total: usize,
    laser_position: Option<[f32; 2]>,
    controller_reset_required: bool,
}

impl RustLaserApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut style = (*cc.egui_ctx.style()).clone();
        style.spacing.item_spacing = egui::vec2(10.0, 8.0);
        cc.egui_ctx.set_style(style);

        Self {
            file_path: String::new(),
            gcode_lines: Vec::new(),
            gcode_polyline: Vec::new(),
            status: String::new(),
            serial_address: DEFAULT_PORT.to_string(),
            serial_baud: DEFAULT_BAUD,
            serial_worker: SerialWorker::spawn(),
            serial_connected: false,
            serial_input: String::new(),
            serial_log: vec!["Serial backend not connected yet.".to_string()],
            jog_step_index: 2,
            jog_feed: 600.0,
            canvas_center: [75.0, 75.0],
            canvas_zoom: 1.0,
            show_millimeter_grid: true,
            show_workspace_grid: true,
            show_workspace_axes: true,
            session_config: SessionConfig {
                gcode_serial_count_mode: GcodeSerialCountMode::RxLogs,
            },
            print_active: false,
            gcode_progress_sent: 0,
            gcode_progress_total: 0,
            laser_position: None,
            controller_reset_required: false,
        }
    }

    fn load_file(&mut self) {
        let path = self.file_path.trim();
        if path.is_empty() {
            self.status = "No file selected.".to_string();
            return;
        }

        match fs::read_to_string(path) {
            Ok(content) => {
                self.gcode_lines = content.lines().map(|line| line.to_string()).collect();
                self.gcode_polyline = gcode_to_polyline(&self.gcode_lines);
                self.status = format!(
                    "Loaded {} line(s), {} plotted point(s).",
                    self.gcode_lines.len(),
                    self.gcode_polyline.len()
                );
            }
            Err(err) => {
                self.gcode_lines.clear();
                self.gcode_polyline.clear();
                self.status = format!("Failed to load file: {err}");
            }
        }
    }

    fn jog_step_value(&self) -> f32 {
        let index = self
            .jog_step_index
            .clamp(0, (JOG_STEP_LUT.len() - 1) as i32) as usize;
        JOG_STEP_LUT[index]
    }

    fn jog_move(&mut self, dx: f32, dy: f32) {
        let step = self.jog_step_value();
        let cmd = SerialCommand::JogMove {
            dx,
            dy,
            step,
            feed: self.jog_feed,
        };
        if let Err(err) = self.serial_worker.send(cmd) {
            self.serial_log.push(format!("Jog failed: {err}"));
        }
    }

    fn jog_stop(&mut self) {
        self.controller_reset_required = true;
        if self.is_printing() {
            self.serial_worker.request_stop();
        } else if let Err(err) = self.serial_worker.send(SerialCommand::JogStop) {
            self.serial_log.push(format!("Stop failed: {err}"));
        }
    }

    fn send_loaded_gcode(&mut self) {
        self.controller_reset_required = false;
        self.print_active = true;
        self.serial_log.push(format!(
            "[Printing G-Code] gcode serial count:{}",
            self.session_config.gcode_serial_count_mode.label()
        ));
        self.gcode_progress_sent = 0;
        self.gcode_progress_total = self.gcode_lines.len();
        self.laser_position = None;
        if let Err(err) = self
            .serial_worker
            .send(SerialCommand::SendLoadedGcode {
                lines: self.gcode_lines.clone(),
                session_config: self.session_config,
            })
        {
            self.serial_log.push(format!("Send failed: {err}"));
        }
    }

    fn reset_to_origin(&mut self) {
        if let Err(err) = self.serial_worker.send(SerialCommand::ResetToOrigin) {
            self.serial_log.push(format!("Reset failed: {err}"));
        }
    }

    fn reset_workspace_view(&mut self) {
        self.canvas_center = [75.0, 75.0];
        self.canvas_zoom = 1.0;
    }

    fn handle_print_aborted(&mut self) {
        self.print_active = false;
        self.gcode_progress_sent = 0;
        self.gcode_progress_total = 0;
        self.laser_position = None;
        self.status =
            "Job stopped. Controller is likely in feed-hold; flush/reset to start a new session."
                .to_string();
    }

    fn flush_controller(&mut self) {
        if let Err(err) = self.serial_worker.send(SerialCommand::FlushController) {
            self.serial_log.push(format!("Flush failed: {err}"));
        } else {
            self.status = "Controller session reset. Ready for new commands or G-code.".to_string();
            self.controller_reset_required = false;
        }
    }

    fn continue_program(&mut self) {
        if let Err(err) = self.serial_worker.send(SerialCommand::ContinueProgram) {
            self.serial_log.push(format!("Continue failed: {err}"));
        } else {
            self.print_active = true;
            self.status = "Continue sent. Controller resumed from hold.".to_string();
            self.controller_reset_required = false;
        }
    }

    fn is_printing(&self) -> bool {
        self.print_active
    }
}

impl eframe::App for RustLaserApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        while let Some(event) = self.serial_worker.try_recv() {
            match event {
                SerialEvent::Log(line) => self.serial_log.push(line),
                SerialEvent::Connected(state) => self.serial_connected = state,
                SerialEvent::PrintStarted => self.print_active = true,
                SerialEvent::PrintFinished => self.print_active = false,
                SerialEvent::GcodeProgress { sent, total } => {
                    self.gcode_progress_sent = sent;
                    self.gcode_progress_total = total;
                }
                SerialEvent::PrintAborted => self.handle_print_aborted(),
                SerialEvent::LaserPosition { position } => {
                    self.laser_position = Some(position);
                }
            }
        }

        if self.is_printing() {
            ctx.request_repaint_after(Duration::from_millis(200));
        }

        egui::TopBottomPanel::top("top_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("Settings", |ui| {
                    ui.checkbox(&mut self.show_millimeter_grid, "Millimeter grid");
                    ui.checkbox(&mut self.show_workspace_grid, "Show workspace grid");
                    ui.checkbox(&mut self.show_workspace_axes, "Show workspace axes");
                    ui.menu_button("gcode serial count", |ui| {
                        ui.selectable_value(
                            &mut self.session_config.gcode_serial_count_mode,
                            GcodeSerialCountMode::CharCount,
                            "char count",
                        );
                        ui.selectable_value(
                            &mut self.session_config.gcode_serial_count_mode,
                            GcodeSerialCountMode::RxLogs,
                            "RX logs",
                        );
                    });
                    ui.label(format!(
                        "gcode serial count: {}",
                        self.session_config.gcode_serial_count_mode.label()
                    ));
                    ui.separator();
                    if ui.button("Reset workspace view").clicked() {
                        self.reset_workspace_view();
                        ui.close_menu();
                    }
                });
            });
        });

        egui::TopBottomPanel::bottom("console_panel")
            .resizable(true)
            .default_height(170.0)
            .min_height(140.0)
            .show(ctx, |ui| {
                ui.columns(3, |columns| {
                    columns[0].heading("Serial GUI");
                    columns[0].separator();
                    columns[0].with_layout(egui::Layout::top_down(egui::Align::Min), |ui| {
                        let input_row_height = 30.0;
                        let reserved_bottom = 44.0;
                        let history_height =
                            (ui.available_height() - input_row_height - reserved_bottom).max(40.0);

                        egui::ScrollArea::vertical()
                            .max_height(history_height)
                            .auto_shrink([false, false])
                            .show(ui, |ui| {
                                let mut history = self.serial_log.join("\n");
                                ui.add_sized(
                                    [ui.available_width(), history_height],
                                    egui::TextEdit::multiline(&mut history)
                                        .desired_width(f32::INFINITY)
                                        .font(egui::TextStyle::Monospace)
                                        .interactive(false),
                                );
                            });

                        ui.separator();
                        ui.horizontal(|ui| {
                            ui.scope(|ui| {
                                ui.visuals_mut().widgets.inactive.bg_fill =
                                    egui::Color32::from_gray(45);
                                ui.visuals_mut().widgets.hovered.bg_fill =
                                    egui::Color32::from_gray(52);
                                ui.visuals_mut().widgets.active.bg_fill =
                                    egui::Color32::from_gray(58);
                                ui.add_sized(
                                    [ui.available_width() - 64.0, input_row_height],
                                    egui::TextEdit::singleline(&mut self.serial_input),
                                );
                            });
                            if ui.button("Send").clicked() {
                                let msg = self.serial_input.trim();
                                if !msg.is_empty() {
                                    match self
                                        .serial_worker
                                        .send(SerialCommand::SendLine(msg.to_string()))
                                    {
                                        Ok(()) => {}
                                        Err(err) => {
                                            self.serial_log.push(format!("Send failed: {err}"))
                                        }
                                    }
                                    self.serial_input.clear();
                                }
                            }
                        });
                    });

                    columns[1].heading("Connection");
                    columns[1].separator();
                    columns[1].label("Serial address");
                    columns[1].text_edit_singleline(&mut self.serial_address);
                    columns[1].label(format!("Baud: {}", self.serial_baud));
                    if columns[1].button("Connect").clicked() {
                        if self.serial_connected {
                            if let Err(err) = self.serial_worker.send(SerialCommand::Disconnect) {
                                self.serial_log.push(format!("Disconnect failed: {err}"));
                            }
                        } else {
                            if let Err(err) = self.serial_worker.send(SerialCommand::Connect {
                                address: self.serial_address.clone(),
                                baud: self.serial_baud,
                            }) {
                                self.serial_log.push(format!("Connect failed: {err}"));
                            }
                        }
                    }
                    columns[1].horizontal(|ui| {
                        if ui.button("Send").clicked() {
                            self.send_loaded_gcode();
                        }
                        if ui.button("Reset").clicked() {
                            self.reset_to_origin();
                        }
                    });
                    if self.controller_reset_required {
                        columns[1].add_space(6.0);
                        columns[1].scope(|ui| {
                            ui.visuals_mut().widgets.inactive.bg_fill =
                                egui::Color32::from_rgb(110, 45, 35);
                            ui.visuals_mut().widgets.hovered.bg_fill =
                                egui::Color32::from_rgb(135, 55, 42);
                            if ui.button("Flush / Reset").clicked() {
                                self.flush_controller();
                            }
                        });
                        columns[1].label("Controller is held after Stop. Reset to continue.");
                    }
                    columns[1].label(if self.serial_connected {
                        "Status: Connected"
                    } else {
                        "Status: Disconnected"
                    });

                    columns[2].heading("Progress");
                    columns[2].separator();
                    columns[2].label("G-code lines sent");
                    let progress = if self.gcode_progress_total > 0 {
                        self.gcode_progress_sent as f32 / self.gcode_progress_total as f32
                    } else {
                        0.0
                    };
                    columns[2].scope(|ui| {
                        ui.visuals_mut().widgets.inactive.bg_fill = egui::Color32::from_gray(56);
                        let progress_percent = progress * 100.0;
                        ui.add(
                            egui::ProgressBar::new(progress)
                                .desired_width(ui.available_width())
                                .text(format!(
                                    "{} / {} ({progress_percent:.0}%)",
                                    self.gcode_progress_sent, self.gcode_progress_total
                                )),
                        );
                    });
                });
            });

        egui::SidePanel::left("left_controls")
            .resizable(true)
            .default_width(340.0)
            .min_width(180.0)
            .show(ctx, |ui| {
                ui.heading("Move");
                ui.separator();
                ui.label("Step");
                ui.scope(|ui| {
                    ui.add_space(20.0);
                    ui.spacing_mut().slider_width = 300.0;
                    ui.add(
                        egui::Slider::new(&mut self.jog_step_index, 0..=3)
                            .step_by(1.0)
                            .show_value(false),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("0.01");
                    ui.add_space(64.0);
                    ui.label("0.1");
                    ui.add_space(64.0);
                    ui.label("1.0");
                    ui.add_space(64.0);
                    ui.label("10.0");
                });
                ui.add(
                    egui::Slider::new(&mut self.jog_feed, 50.0..=3000.0)
                        .text("Feed (mm/min)")
                        .step_by(10.0),
                );
                ui.add_space(2.0);

                let button_size = egui::vec2(56.0, 36.0);
                let gap = 6.0;

                ui.add_space(8.0);
                ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                    egui::Grid::new("move_cross_grid")
                        .spacing(egui::vec2(gap, gap))
                        .show(ui, |ui| {
                            ui.add_sized(button_size, egui::Label::new(""));
                            ui.add_sized(button_size, egui::Label::new(""));
                            if ui
                                .add_sized(
                                    button_size,
                                    egui::Button::new(pad_left_panel_button_label("Up")),
                                )
                                .clicked()
                            {
                                self.jog_move(0.0, 1.0);
                            }
                            ui.add_sized(button_size, egui::Label::new(""));
                            ui.end_row();

                            ui.add_sized(button_size, egui::Label::new(""));
                            if ui
                                .add_sized(
                                    button_size,
                                    egui::Button::new(pad_left_panel_button_label("Left")),
                                )
                                .clicked()
                            {
                                self.jog_move(-1.0, 0.0);
                            }
                            if ui
                                .add_sized(
                                    button_size,
                                    if self.is_printing() {
                                        egui::Button::new(
                                            egui::RichText::new(pad_left_panel_button_label("Stop"))
                                                .strong()
                                                .color(egui::Color32::WHITE),
                                        )
                                        .fill(egui::Color32::from_rgb(180, 40, 40))
                                    } else {
                                        egui::Button::new(pad_left_panel_button_label("Stop"))
                                    },
                                )
                                .clicked()
                            {
                                self.jog_stop();
                            }
                            if ui
                                .add_sized(
                                    button_size,
                                    egui::Button::new(pad_left_panel_button_label("Right")),
                                )
                                .clicked()
                            {
                                self.jog_move(1.0, 0.0);
                            }
                            ui.end_row();

                            ui.add_sized(button_size, egui::Label::new(""));
                            ui.add_sized(button_size, egui::Label::new(""));
                            if ui
                                .add_sized(
                                    button_size,
                                    egui::Button::new(pad_left_panel_button_label("Down")),
                                )
                                .clicked()
                            {
                                self.jog_move(0.0, -1.0);
                            }
                            ui.add_sized(button_size, egui::Label::new(""));
                        });
                });

                ui.add_space(14.0);
                ui.label("Extra Keys");
                let matrix_button_size = egui::vec2(92.0, 30.0);
                let matrix_gap = 4.0;

                ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                    egui::Grid::new("inner_3x3_in_5x5")
                        .spacing(egui::vec2(matrix_gap, matrix_gap))
                        .show(ui, |ui| {
                            for row in 0..5 {
                                for col in 0..5 {
                                    let is_boundary = row == 0 || row == 4 || col == 0 || col == 4;
                                    if is_boundary {
                                        ui.add_sized(matrix_button_size, egui::Label::new(""));
                                    } else {
                                        let button_id = (row - 1) * 3 + col;
                                        let label = if button_id == 1 {
                                            pad_left_panel_button_label("Home")
                                        } else if button_id == 2 {
                                            pad_left_panel_button_label("SReset")
                                        } else if button_id == 3 {
                                            pad_left_panel_button_label("Continue")
                                        } else {
                                            pad_left_panel_button_label(&format!(
                                                "Button {button_id}"
                                            ))
                                        };
                                        let clicked = if button_id == 2
                                            && self.controller_reset_required
                                        {
                                            ui.add_sized(
                                                matrix_button_size,
                                                egui::Button::new(
                                                    egui::RichText::new(label.as_str())
                                                        .strong()
                                                        .color(egui::Color32::BLACK),
                                                )
                                                .fill(egui::Color32::from_rgb(255, 215, 64))
                                                .stroke(egui::Stroke::new(
                                                    1.5,
                                                    egui::Color32::from_rgb(120, 100, 20),
                                                )),
                                            )
                                            .clicked()
                                        } else if button_id == 3
                                            && self.controller_reset_required
                                        {
                                            ui.add_sized(
                                                matrix_button_size,
                                                egui::Button::new(
                                                    egui::RichText::new(label.as_str())
                                                        .strong()
                                                        .color(egui::Color32::WHITE),
                                                )
                                                .fill(egui::Color32::from_rgb(32, 160, 80))
                                                .stroke(egui::Stroke::new(
                                                    1.5,
                                                    egui::Color32::from_rgb(16, 96, 48),
                                                )),
                                            )
                                            .clicked()
                                        } else {
                                            ui.add_sized(
                                                matrix_button_size,
                                                egui::Button::new(label.as_str()),
                                            )
                                            .clicked()
                                        };
                                        if clicked {
                                            if button_id == 1 {
                                                if let Err(err) =
                                                    self.serial_worker.send(SerialCommand::Home {
                                                        feed: self.jog_feed,
                                                    })
                                                {
                                                    self.serial_log
                                                        .push(format!("Home failed: {err}"));
                                                }
                                            } else if button_id == 2 {
                                                if self.controller_reset_required {
                                                    self.flush_controller();
                                                }
                                            } else if button_id == 3 {
                                                if self.controller_reset_required {
                                                    self.continue_program();
                                                }
                                            } else {
                                                self.serial_log.push(format!(
                                                    "Matrix button ({row},{col}) pressed"
                                                ));
                                            }
                                        }
                                    }
                                }
                                ui.end_row();
                            }
                        });
                });

            });

        egui::SidePanel::right("gcode_panel")
            .resizable(true)
            .default_width(380.0)
            .min_width(260.0)
            .show(ctx, |ui| {
                ui.heading("G-code");
                ui.separator();

                ui.horizontal(|ui| {
                    ui.label("File");
                    ui.scope(|ui| {
                        ui.visuals_mut().widgets.inactive.bg_fill = egui::Color32::from_gray(45);
                        ui.visuals_mut().widgets.hovered.bg_fill = egui::Color32::from_gray(52);
                        ui.visuals_mut().widgets.active.bg_fill = egui::Color32::from_gray(58);
                        ui.text_edit_singleline(&mut self.file_path);
                    });
                    if ui.button("Open").clicked() {
                        if let Some(path) = FileDialog::new()
                            .add_filter("G-code", &["gcode", "nc"])
                            .pick_file()
                        {
                            self.file_path = path.display().to_string();
                            self.load_file();
                        }
                    }
                    if ui.button("Load").clicked() {
                        self.load_file();
                    }
                });

                if !self.status.is_empty() {
                    ui.label(&self.status);
                }

                ui.separator();
                egui::Frame::none()
                    .fill(egui::Color32::from_gray(58))
                    .show(ui, |ui| {
                        egui::ScrollArea::both()
                            .auto_shrink([false, false])
                            .show(ui, |ui| {
                                for (idx, line) in self.gcode_lines.iter().enumerate() {
                                    let row_color = if idx % 2 == 0 {
                                        egui::Color32::from_gray(58)
                                    } else {
                                        egui::Color32::from_gray(44)
                                    };

                                    egui::Frame::none().fill(row_color).show(ui, |ui| {
                                        ui.set_min_width(ui.available_width());
                                        ui.horizontal(|ui| {
                                            ui.add_space(6.0);
                                            ui.monospace(line);
                                        });
                                    });
                                }
                            });
                    });
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            let x_max = 150.0_f32;
            let y_max = 150.0_f32;

            ui.heading("Workspace");
            ui.separator();

            let margin = 28.0;
            let size = ui.available_size();
            let (rect, response) = ui.allocate_exact_size(size, egui::Sense::drag());
            let painter = ui.painter_at(rect);

            let frame = rect.shrink(margin);
            let side = frame.width().min(frame.height());
            let plot = egui::Rect::from_min_size(
                egui::pos2(
                    frame.left() + (frame.width() - side) * 0.5,
                    frame.top() + (frame.height() - side) * 0.5,
                ),
                egui::vec2(side, side),
            );

            let base_scale = side / x_max;
            let mut scale = base_scale * self.canvas_zoom;

            if response.dragged() {
                let drag = ctx.input(|i| i.pointer.delta());
                self.canvas_center[0] -= drag.x / scale;
                self.canvas_center[1] += drag.y / scale;
            }

            if response.hovered() {
                let scroll_y = ctx.input(|i| i.smooth_scroll_delta.y);
                if scroll_y.abs() > f32::EPSILON {
                    let zoom_factor = (1.0 + scroll_y * 0.0015).clamp(0.80, 1.25);
                    if let Some(pointer) = response.hover_pos() {
                        let world_before =
                            screen_to_world(pointer, plot, self.canvas_center, scale);
                        self.canvas_zoom = (self.canvas_zoom * zoom_factor).clamp(0.2, 30.0);
                        scale = base_scale * self.canvas_zoom;

                        let offset_x = (pointer.x - plot.center().x) / scale;
                        let offset_y = -(pointer.y - plot.center().y) / scale;
                        self.canvas_center[0] = world_before[0] - offset_x;
                        self.canvas_center[1] = world_before[1] - offset_y;
                    }
                }
            }

            let border = egui::Stroke::new(1.0, egui::Color32::from_gray(170));
            painter.line_segment([plot.left_top(), plot.right_top()], border);
            painter.line_segment([plot.right_top(), plot.right_bottom()], border);
            painter.line_segment([plot.right_bottom(), plot.left_bottom()], border);
            painter.line_segment([plot.left_bottom(), plot.left_top()], border);

            let clipped = painter.with_clip_rect(plot);

            if self.show_millimeter_grid {
                let zoom_boost = (self.canvas_zoom - 1.0).max(0.0);
                let fine_grid_alpha = (2.0 + zoom_boost * 2.5)
                    .round()
                    .clamp(2.0, 18.0) as u8;
                let fine_grid_stroke =
                    egui::Stroke::new(1.0, egui::Color32::from_white_alpha(fine_grid_alpha));
                for step in 0..=(x_max as i32) {
                    let x = step as f32;
                    let x_top = world_to_screen([x, 0.0], plot, self.canvas_center, scale);
                    let x_bottom = world_to_screen([x, y_max], plot, self.canvas_center, scale);
                    clipped.line_segment([x_top, x_bottom], fine_grid_stroke);
                }
                for step in 0..=(y_max as i32) {
                    let y = step as f32;
                    let y_left = world_to_screen([0.0, y], plot, self.canvas_center, scale);
                    let y_right = world_to_screen([x_max, y], plot, self.canvas_center, scale);
                    clipped.line_segment([y_left, y_right], fine_grid_stroke);
                }
            }

            if self.show_workspace_grid {
                let grid_stroke = egui::Stroke::new(1.0, egui::Color32::from_white_alpha(40));
                for i in 0..=10 {
                    let x = i as f32 * (x_max / 10.0);
                    let y = i as f32 * (y_max / 10.0);
                    let x_top = world_to_screen([x, 0.0], plot, self.canvas_center, scale);
                    let x_bottom = world_to_screen([x, y_max], plot, self.canvas_center, scale);
                    let y_left = world_to_screen([0.0, y], plot, self.canvas_center, scale);
                    let y_right = world_to_screen([x_max, y], plot, self.canvas_center, scale);
                    clipped.line_segment([x_top, x_bottom], grid_stroke);
                    clipped.line_segment([y_left, y_right], grid_stroke);
                }
            }

            if self.show_workspace_axes {
                let axis_stroke = egui::Stroke::new(1.5, egui::Color32::from_gray(220));
                clipped.line_segment(
                    [
                        world_to_screen([0.0, 0.0], plot, self.canvas_center, scale),
                        world_to_screen([x_max, 0.0], plot, self.canvas_center, scale),
                    ],
                    axis_stroke,
                );
                clipped.line_segment(
                    [
                        world_to_screen([0.0, 0.0], plot, self.canvas_center, scale),
                        world_to_screen([0.0, y_max], plot, self.canvas_center, scale),
                    ],
                    axis_stroke,
                );

                let tick_font = egui::TextStyle::Small.resolve(ui.style());
                let tick_color = egui::Color32::from_gray(210);
                let tick_stroke = egui::Stroke::new(1.0, tick_color);

                for i in 0..=10 {
                    let x = i as f32 * (x_max / 10.0);
                    let y = i as f32 * (y_max / 10.0);

                    let x_tick = world_to_screen([x, 0.0], plot, self.canvas_center, scale);
                    clipped.line_segment(
                        [
                            egui::pos2(x_tick.x, x_tick.y - 4.0),
                            egui::pos2(x_tick.x, x_tick.y + 4.0),
                        ],
                        tick_stroke,
                    );
                    painter.text(
                        egui::pos2(x_tick.x, plot.bottom() + 6.0),
                        egui::Align2::CENTER_TOP,
                        format!("{x:.0}"),
                        tick_font.clone(),
                        tick_color,
                    );

                    let y_tick = world_to_screen([0.0, y], plot, self.canvas_center, scale);
                    clipped.line_segment(
                        [
                            egui::pos2(y_tick.x - 4.0, y_tick.y),
                            egui::pos2(y_tick.x + 4.0, y_tick.y),
                        ],
                        tick_stroke,
                    );
                    painter.text(
                        egui::pos2(plot.left() - 6.0, y_tick.y),
                        egui::Align2::RIGHT_CENTER,
                        format!("{y:.0}"),
                        tick_font.clone(),
                        tick_color,
                    );
                }
            }

            if self.gcode_polyline.len() >= 2 {
                let path_stroke = egui::Stroke::new(2.0, egui::Color32::from_rgb(55, 145, 255));
                for segment in self.gcode_polyline.windows(2) {
                    let a = world_to_screen(segment[0], plot, self.canvas_center, scale);
                    let b = world_to_screen(segment[1], plot, self.canvas_center, scale);
                    clipped.line_segment([a, b], path_stroke);
                }
            } else if let Some(point) = self.gcode_polyline.first() {
                let p = world_to_screen(*point, plot, self.canvas_center, scale);
                clipped.circle_filled(p, 2.5, egui::Color32::from_rgb(55, 145, 255));
            }

            let show_laser_marker = self.session_config.gcode_serial_count_mode
                == GcodeSerialCountMode::RxLogs
                && self.is_printing();
            if show_laser_marker {
                if let Some(position) = self.laser_position {
                    let p = world_to_screen(position, plot, self.canvas_center, scale);
                    let marker_color = egui::Color32::from_rgb(255, 120, 80);
                    let marker_radius = 7.0;
                    clipped.circle_stroke(p, marker_radius, egui::Stroke::new(2.0, marker_color));
                    clipped.line_segment(
                        [egui::pos2(p.x - 5.0, p.y), egui::pos2(p.x + 5.0, p.y)],
                        egui::Stroke::new(2.0, marker_color),
                    );
                    clipped.line_segment(
                        [egui::pos2(p.x, p.y - 5.0), egui::pos2(p.x, p.y + 5.0)],
                        egui::Stroke::new(2.0, marker_color),
                    );
                }
            }

            painter.text(
                plot.left_top() + egui::vec2(8.0, 8.0),
                egui::Align2::LEFT_TOP,
                format!(
                    "Pan: drag | Zoom: wheel\nCenter: ({:.2}, {:.2})  Zoom: {:.2}x",
                    self.canvas_center[0], self.canvas_center[1], self.canvas_zoom
                ),
                egui::TextStyle::Small.resolve(ui.style()),
                egui::Color32::from_gray(210),
            );
        });
    }
}

fn world_to_screen(
    point: [f32; 2],
    plot: egui::Rect,
    canvas_center: [f32; 2],
    scale: f32,
) -> egui::Pos2 {
    egui::pos2(
        plot.center().x + (point[0] - canvas_center[0]) * scale,
        plot.center().y - (point[1] - canvas_center[1]) * scale,
    )
}

fn screen_to_world(
    point: egui::Pos2,
    plot: egui::Rect,
    canvas_center: [f32; 2],
    scale: f32,
) -> [f32; 2] {
    [
        canvas_center[0] + (point.x - plot.center().x) / scale,
        canvas_center[1] - (point.y - plot.center().y) / scale,
    ]
}

fn pad_left_panel_button_label(label: &str) -> String {
    let len = label.chars().count();
    if len >= LEFT_PANEL_BUTTON_LABEL_WIDTH {
        return label.to_string();
    }

    let total_padding = LEFT_PANEL_BUTTON_LABEL_WIDTH - len;
    let left_padding = total_padding / 2;
    let right_padding = total_padding - left_padding;

    format!(
        "{}{}{}",
        " ".repeat(left_padding),
        label,
        " ".repeat(right_padding)
    )
}

fn gcode_to_polyline(lines: &[String]) -> Vec<[f32; 2]> {
    let mut points = Vec::new();
    let mut current_x = 0.0_f32;
    let mut current_y = 0.0_f32;
    let mut has_point = false;

    for raw in lines {
        let line = raw.split(';').next().unwrap_or("").trim().to_uppercase();
        if line.is_empty() {
            continue;
        }

        let mut changed = false;
        if let Some(x) = extract_axis_value(&line, 'X') {
            current_x = x;
            changed = true;
        }
        if let Some(y) = extract_axis_value(&line, 'Y') {
            current_y = y;
            changed = true;
        }

        if changed {
            if !has_point {
                points.push([current_x, current_y]);
                has_point = true;
            } else if let Some(prev) = points.last() {
                if (prev[0] - current_x).abs() > f32::EPSILON
                    || (prev[1] - current_y).abs() > f32::EPSILON
                {
                    points.push([current_x, current_y]);
                }
            }
        }
    }

    points
}

fn extract_axis_value(line: &str, axis: char) -> Option<f32> {
    let pos = line.find(axis)?;
    let tail = &line[pos + 1..];
    let mut value = String::new();

    for ch in tail.chars() {
        if ch.is_ascii_digit() || ch == '.' || ch == '-' || ch == '+' {
            value.push(ch);
        } else {
            break;
        }
    }

    if value.is_empty() {
        None
    } else {
        value.parse::<f32>().ok()
    }
}
