mod modules;

use eframe::egui;
use modules::serial_lib::{DEFAULT_BAUD, DEFAULT_PORT};
use modules::serial_worker::{
    GcodeSerialCountMode, SerialCommand, SerialEvent, SerialWorker, SessionConfig,
};
use rfd::FileDialog;
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::PathBuf;
use std::process::Command;
use std::time::Duration;

const JOG_STEP_LUT: [f32; 4] = [0.01, 0.1, 1.0, 10.0];
const LEFT_PANEL_BUTTON_LABEL_WIDTH: usize = "Button 0".len();

fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("RustLaser GUI v0.4")
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
    loop_active: bool,
    loop_selecting: bool,
    loop_selection_anchor: Option<[f32; 2]>,
    loop_rectangle: Option<[[f32; 2]; 2]>,
    gcode_progress_sent: usize,
    gcode_progress_total: usize,
    rx_buffer_used: usize,
    rx_buffer_capacity: usize,
    laser_position: Option<[f32; 2]>,
    controller_reset_required: bool,
    is_debug_mode: bool,
    telemetry_enabled: bool,
    telemetry_log_path: PathBuf,
    rx_fallback_prompt_open: bool,
}

impl RustLaserApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut style = (*cc.egui_ctx.style()).clone();
        style.spacing.item_spacing = egui::vec2(10.0, 8.0);
        cc.egui_ctx.set_style(style);
        let telemetry_log_path = Self::build_telemetry_log_path();

        let mut app = Self {
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
                no_rx_prompt_exit: true,
            },
            print_active: false,
            loop_active: false,
            loop_selecting: false,
            loop_selection_anchor: None,
            loop_rectangle: None,
            gcode_progress_sent: 0,
            gcode_progress_total: 0,
            rx_buffer_used: 0,
            rx_buffer_capacity: 127,
            laser_position: None,
            controller_reset_required: false,
            is_debug_mode: false,
            telemetry_enabled: true,
            telemetry_log_path,
            rx_fallback_prompt_open: false,
        };
        app.append_telemetry_log("STARTUP", "Rust Laser GUI Started!");
        app
    }

    fn build_telemetry_log_path() -> PathBuf {
        let timestamp = Command::new("date")
            .arg("+%F_%H-%M-%S")
            .output()
            .ok()
            .and_then(|output| {
                if output.status.success() {
                    Some(String::from_utf8_lossy(&output.stdout).trim().to_string())
                } else {
                    None
                }
            })
            .filter(|value| !value.is_empty())
            .unwrap_or_else(|| "unknown_date_unknown_hour".to_string());

        PathBuf::from("Rust_Laser_GUI_Telemetry")
            .join(format!("rust_laser_gui_telemetry_{timestamp}.log"))
    }

    fn telemetry_time_hms() -> String {
        Command::new("date")
            .arg("+%H:%M:%S")
            .output()
            .ok()
            .and_then(|output| {
                if output.status.success() {
                    Some(String::from_utf8_lossy(&output.stdout).trim().to_string())
                } else {
                    None
                }
            })
            .filter(|value| !value.is_empty())
            .unwrap_or_else(|| "unknown_time".to_string())
    }

    fn append_telemetry_log(&mut self, label: &str, message: &str) {
        if !self.telemetry_enabled {
            return;
        }

        if let Some(parent) = self.telemetry_log_path.parent() {
            if let Err(err) = fs::create_dir_all(parent) {
                self.status = format!("Telemetry setup failed: {err}");
                return;
            }
        }

        match OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.telemetry_log_path)
        {
            Ok(mut file) => {
                let time_only = Self::telemetry_time_hms();
                if let Err(err) = writeln!(file, "[{label}][{time_only}]: {message}") {
                    self.status = format!("Telemetry write failed: {err}");
                    return;
                }
                let variables_json = self.telemetry_variables_json();
                if let Err(err) = writeln!(file, "[VARIABLES][{time_only}]: {variables_json}") {
                    self.status = format!("Telemetry write failed: {err}");
                    return;
                }
            }
            Err(err) => {
                self.status = format!("Telemetry open failed: {err}");
                return;
            }
        }
    }

    fn telemetry_variables_json(&self) -> String {
        format!(
            concat!(
                "{{",
                "\"status\":\"{}\",",
                "\"serial_connected\":{},",
                "\"serial_address\":\"{}\",",
                "\"serial_baud\":{},",
                "\"print_active\":{},",
                "\"loop_active\":{},",
                "\"loop_selecting\":{},",
                "\"gcode_lines\":{},",
                "\"gcode_progress_sent\":{},",
                "\"gcode_progress_total\":{},",
                "\"rx_buffer_used\":{},",
                "\"rx_buffer_capacity\":{},",
                "\"controller_reset_required\":{},",
                "\"telemetry_enabled\":{},",
                "\"show_millimeter_grid\":{},",
                "\"show_workspace_grid\":{},",
                "\"show_workspace_axes\":{},",
                "\"gcode_serial_count_mode\":\"{}\",",
                "\"no_rx_prompt_exit\":{},",
                "\"rx_fallback_prompt_open\":{},",
                "\"jog_step_index\":{},",
                "\"jog_feed\":{},",
                "\"canvas_center\":[{},{}],",
                "\"canvas_zoom\":{},",
                "\"laser_position\":{},",
                "\"file_path\":\"{}\"",
                "}}"
            ),
            Self::json_escape(&self.status),
            self.serial_connected,
            Self::json_escape(&self.serial_address),
            self.serial_baud,
            self.print_active,
            self.loop_active,
            self.loop_selecting,
            self.gcode_lines.len(),
            self.gcode_progress_sent,
            self.gcode_progress_total,
            self.rx_buffer_used,
            self.rx_buffer_capacity,
            self.controller_reset_required,
            self.telemetry_enabled,
            self.show_millimeter_grid,
            self.show_workspace_grid,
            self.show_workspace_axes,
            self.session_config.gcode_serial_count_mode.label(),
            self.session_config.no_rx_prompt_exit,
            self.rx_fallback_prompt_open,
            self.jog_step_index,
            self.jog_feed,
            self.canvas_center[0],
            self.canvas_center[1],
            self.canvas_zoom,
            Self::telemetry_optional_position_json(self.laser_position),
            Self::json_escape(&self.file_path),
        )
    }

    fn telemetry_optional_position_json(position: Option<[f32; 2]>) -> String {
        match position {
            Some([x, y]) => format!("[{x},{y}]"),
            None => "null".to_string(),
        }
    }

    fn json_escape(value: &str) -> String {
        value
            .replace('\\', "\\\\")
            .replace('"', "\\\"")
            .replace('\n', "\\n")
            .replace('\r', "\\r")
            .replace('\t', "\\t")
    }

    fn load_file(&mut self) {
        let path = self.file_path.trim().to_string();
        if path.is_empty() {
            self.status = "No file selected.".to_string();
            self.append_telemetry_log("FILE", "Load requested with empty file path");
            return;
        }

        self.append_telemetry_log("FILE", &format!("Loading file: {path}"));

        match fs::read_to_string(&path) {
            Ok(content) => {
                self.gcode_lines = content.lines().map(|line| line.to_string()).collect();
                self.gcode_polyline = gcode_to_polyline(&self.gcode_lines);
                self.status = format!(
                    "Loaded {} line(s), {} plotted point(s).",
                    self.gcode_lines.len(),
                    self.gcode_polyline.len()
                );
                self.append_telemetry_log(
                    "FILE",
                    &format!(
                        "Loaded file successfully: {} line(s), {} plotted point(s)",
                        self.gcode_lines.len(),
                        self.gcode_polyline.len()
                    ),
                );
            }
            Err(err) => {
                self.gcode_lines.clear();
                self.gcode_polyline.clear();
                self.status = format!("Failed to load file: {err}");
                self.append_telemetry_log("ERROR", &format!("File load failed: {err}"));
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
        self.append_telemetry_log(
            "ACTION",
            &format!(
                "Jog move requested: dx={dx:.3}, dy={dy:.3}, step={step:.3}, feed={:.0}",
                self.jog_feed
            ),
        );
        if let Err(err) = self.serial_worker.send(cmd) {
            self.serial_log.push(format!("Jog failed: {err}"));
            self.append_telemetry_log("ERROR", &format!("Jog command send failed: {err}"));
        }
    }

    fn jog_stop(&mut self) {
        self.controller_reset_required = true;
        self.append_telemetry_log(
            "ACTION",
            &format!("Stop requested while print_active={}", self.is_printing()),
        );
        if self.is_printing() {
            self.serial_worker.request_stop();
        } else if let Err(err) = self.serial_worker.send(SerialCommand::JogStop) {
            self.serial_log.push(format!("Stop failed: {err}"));
            self.append_telemetry_log("ERROR", &format!("Jog stop send failed: {err}"));
        }
    }

    fn send_loaded_gcode(&mut self) {
        self.controller_reset_required = false;
        self.print_active = true;
        self.append_telemetry_log(
            "ACTION",
            &format!(
                "Send loaded G-code requested: {} line(s), mode={}",
                self.gcode_lines.len(),
                self.session_config.gcode_serial_count_mode.label()
            ),
        );
        self.serial_log.push(format!(
            "[Printing G-Code] gcode serial count:{}",
            self.session_config.gcode_serial_count_mode.label()
        ));
        self.gcode_progress_sent = 0;
        self.gcode_progress_total = self.gcode_lines.len();
        self.rx_buffer_used = 0;
        self.laser_position = None;
        self.rx_fallback_prompt_open = false;
        if let Err(err) = self
            .serial_worker
            .send(SerialCommand::SendLoadedGcode {
                lines: self.gcode_lines.clone(),
                session_config: self.session_config,
            })
        {
            self.serial_log.push(format!("Send failed: {err}"));
            self.append_telemetry_log("ERROR", &format!("Send loaded G-code failed: {err}"));
        }
    }

    fn reset_to_origin(&mut self) {
        self.append_telemetry_log("ACTION", "Reset to origin requested");
        if let Err(err) = self.serial_worker.send(SerialCommand::ResetToOrigin) {
            self.serial_log.push(format!("Reset failed: {err}"));
            self.append_telemetry_log("ERROR", &format!("Reset to origin failed: {err}"));
        }
    }

    fn reset_workspace_view(&mut self) {
        self.canvas_center = [75.0, 75.0];
        self.canvas_zoom = 1.0;
        self.append_telemetry_log("GUI", "Workspace view reset");
    }

    fn loop_feed(&self) -> f32 {
        self.jog_feed.clamp(300.0, 1200.0)
    }

    fn toggle_loop_mode(&mut self) {
        if self.loop_selecting {
            self.loop_selecting = false;
            self.loop_selection_anchor = None;
            self.loop_rectangle = None;
            self.status = "Loop selection cancelled.".to_string();
            self.append_telemetry_log("LOOP", "Loop selection cancelled");
            return;
        }

        if self.loop_active {
            self.serial_worker.request_stop();
            self.loop_rectangle = None;
            self.status = "Loop stop requested. Controller reset will be sent automatically."
                .to_string();
            self.append_telemetry_log("LOOP", "Loop stop requested");
            return;
        }

        self.loop_selecting = true;
        self.loop_selection_anchor = None;
        self.loop_rectangle = None;
        self.status =
            "Loop mode armed. Click two diagonal rectangle corners in the workspace.".to_string();
        self.append_telemetry_log("LOOP", "Loop mode armed");
    }

    fn handle_loop_selection_click(&mut self, point: [f32; 2]) {
        if !self.loop_selecting {
            return;
        }

        self.append_telemetry_log(
            "LOOP",
            &format!("Loop selection click at X{:.3} Y{:.3}", point[0], point[1]),
        );

        if let Some(anchor) = self.loop_selection_anchor {
            if (anchor[0] - point[0]).abs() <= f32::EPSILON
                || (anchor[1] - point[1]).abs() <= f32::EPSILON
            {
                self.status =
                    "Loop rectangle needs non-zero width and height. Pick a different second corner."
                        .to_string();
                self.append_telemetry_log(
                    "LOOP",
                    "Loop selection rejected because width or height was zero",
                );
                return;
            }

            let rectangle = [anchor, point];
            self.loop_rectangle = Some(rectangle);
            self.loop_selection_anchor = None;
            self.loop_selecting = false;
            self.loop_active = true;
            self.status = format!(
                "Loop rectangle started at F{:.0}. Press Stop Loop to stop.",
                self.loop_feed()
            );
            self.append_telemetry_log(
                "LOOP",
                &format!(
                    "Loop rectangle confirmed: ({:.3}, {:.3}) -> ({:.3}, {:.3}) at feed {:.0}",
                    rectangle[0][0],
                    rectangle[0][1],
                    rectangle[1][0],
                    rectangle[1][1],
                    self.loop_feed()
                ),
            );
            if let Err(err) = self.serial_worker.send(SerialCommand::LoopRectangle {
                corners: rectangle,
                feed: self.loop_feed(),
            }) {
                self.loop_active = false;
                self.loop_rectangle = None;
                self.serial_log.push(format!("Loop start failed: {err}"));
                self.append_telemetry_log("ERROR", &format!("Loop start failed: {err}"));
            }
        } else {
            self.loop_selection_anchor = Some(point);
            self.loop_rectangle = Some([point, point]);
            self.status = "Loop mode: pick the opposite corner.".to_string();
            self.append_telemetry_log("LOOP", "Loop first corner stored");
        }
    }

    fn handle_print_aborted(&mut self) {
        self.print_active = false;
        self.gcode_progress_sent = 0;
        self.gcode_progress_total = 0;
        self.rx_buffer_used = 0;
        self.laser_position = None;
        self.rx_fallback_prompt_open = false;
        self.status = if self.controller_reset_required {
            "Job stopped. Controller is likely in feed-hold; flush/reset to start a new session."
                .to_string()
        } else {
            "Job stopped. Controller session was reset; ready for a new print.".to_string()
        };
        self.append_telemetry_log("PRINT", "Print aborted event handled by GUI");
    }

    fn flush_controller(&mut self) {
        self.append_telemetry_log("ACTION", "Flush / Reset requested");
        if let Err(err) = self.serial_worker.send(SerialCommand::FlushController) {
            self.serial_log.push(format!("Flush failed: {err}"));
            self.append_telemetry_log("ERROR", &format!("Flush / Reset failed: {err}"));
        } else {
            self.status = "Controller session reset. Ready for new commands or G-code.".to_string();
            self.controller_reset_required = false;
            self.append_telemetry_log("ACTION", "Flush / Reset command sent");
        }
    }

    fn continue_program(&mut self) {
        self.append_telemetry_log("ACTION", "Continue requested");
        if let Err(err) = self.serial_worker.send(SerialCommand::ContinueProgram) {
            self.serial_log.push(format!("Continue failed: {err}"));
            self.append_telemetry_log("ERROR", &format!("Continue failed: {err}"));
        } else {
            self.print_active = true;
            self.status = "Continue sent. Controller resumed from hold.".to_string();
            self.controller_reset_required = false;
            self.append_telemetry_log("ACTION", "Continue command sent");
        }
    }

    fn is_printing(&self) -> bool {
        self.print_active
    }

    fn show_rx_fallback_prompt(&mut self) {
        self.rx_fallback_prompt_open = true;
        self.status = format!(
            "Controller does not report RX buffer fill. Choose Yes to keep tracking until Idle or No to stop the print."
        );
        self.append_telemetry_log(
            "PRINT",
            "RX fallback continue prompt shown",
        );
    }

    fn confirm_rx_fallback_continue(&mut self) {
        self.serial_worker.confirm_fallback_continue();
        self.rx_fallback_prompt_open = false;
        self.status =
            "Fallback continue confirmed. Waiting for the controller to report Idle.".to_string();
        self.append_telemetry_log("PRINT", "RX fallback continue confirmed by user");
    }

    fn reject_rx_fallback_continue(&mut self) {
        self.rx_fallback_prompt_open = false;
        self.append_telemetry_log("PRINT", "RX fallback continue rejected by user");
        self.status = "Stopping print and resetting controller session for a clean next start."
            .to_string();
        self.controller_reset_required = false;
        self.serial_worker.request_stop_with_reset();
    }
}

impl eframe::App for RustLaserApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        while let Some(event) = self.serial_worker.try_recv() {
            match event {
                SerialEvent::Log(line) => {
                    self.append_telemetry_log("WORKER", &format!("Log event: {line}"));
                    self.serial_log.push(line);
                }
                SerialEvent::Connected(state) => {
                    self.serial_connected = state;
                    if !state {
                        self.rx_buffer_used = 0;
                    }
                    self.append_telemetry_log("SERIAL", &format!("Connected event: {state}"));
                }
                SerialEvent::PrintStarted => {
                    self.print_active = true;
                    self.rx_buffer_used = 0;
                    self.status = "Print in progress. Streaming G-code to the controller."
                        .to_string();
                    self.append_telemetry_log("PRINT", "Print started event");
                }
                SerialEvent::PrintFinished => {
                    self.print_active = false;
                    self.rx_buffer_used = 0;
                    self.rx_fallback_prompt_open = false;
                    self.append_telemetry_log("PRINT", "Print finished event");
                }
                SerialEvent::RxFallbackContinuePrompt => {
                    self.show_rx_fallback_prompt();
                }
                SerialEvent::GcodeProgress { sent, total } => {
                    self.gcode_progress_sent = sent;
                    self.gcode_progress_total = total;
                    self.append_telemetry_log(
                        "PROGRESS",
                        &format!("G-code progress updated: {sent}/{total}"),
                    );
                }
                SerialEvent::RxBufferFill { used, capacity } => {
                    self.rx_buffer_used = used;
                    self.rx_buffer_capacity = capacity;
                    self.append_telemetry_log(
                        "PROGRESS",
                        &format!("RX buffer fill updated: {used}/{capacity}"),
                    );
                }
                SerialEvent::LoopStarted => {
                    self.loop_active = true;
                    self.append_telemetry_log("LOOP", "Loop started event");
                }
                SerialEvent::LoopStopped => {
                    self.loop_active = false;
                    self.loop_selecting = false;
                    self.loop_selection_anchor = None;
                    self.controller_reset_required = false;
                    if !self.print_active {
                        self.status = "Loop rectangle stopped.".to_string();
                    }
                    self.append_telemetry_log("LOOP", "Loop stopped event");
                }
                SerialEvent::PrintAborted => self.handle_print_aborted(),
                SerialEvent::LaserPosition { position } => {
                    self.laser_position = Some(position);
                    self.append_telemetry_log(
                        "POSITION",
                        &format!("Laser position updated: X{:.3} Y{:.3}", position[0], position[1]),
                    );
                }
            }
        }

        if self.is_printing() || self.loop_active {
            ctx.request_repaint_after(Duration::from_millis(200));
        }
        egui::TopBottomPanel::top("top_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("Settings", |ui| {
                    if ui
                        .checkbox(&mut self.show_millimeter_grid, "Millimeter grid")
                        .changed()
                    {
                        self.append_telemetry_log(
                            "SETTINGS",
                            &format!("Millimeter grid set to {}", self.show_millimeter_grid),
                        );
                    }
                    if ui
                        .checkbox(&mut self.show_workspace_grid, "Show workspace grid")
                        .changed()
                    {
                        self.append_telemetry_log(
                            "SETTINGS",
                            &format!("Workspace grid set to {}", self.show_workspace_grid),
                        );
                    }
                    if ui
                        .checkbox(&mut self.show_workspace_axes, "Show workspace axes")
                        .changed()
                    {
                        self.append_telemetry_log(
                            "SETTINGS",
                            &format!("Workspace axes set to {}", self.show_workspace_axes),
                        );
                    }
                    if ui
                        .checkbox(
                            &mut self.session_config.no_rx_prompt_exit,
                            "No RX Prompt Exit",
                        )
                        .changed()
                    {
                        self.append_telemetry_log(
                            "SETTINGS",
                            &format!(
                                "No RX Prompt Exit set to {}",
                                self.session_config.no_rx_prompt_exit
                            ),
                        );
                    }
                    if ui.checkbox(&mut self.is_debug_mode, "Debug Mode").changed() {
                        self.append_telemetry_log(
                            "SETTINGS",
                            &format!("Debug Mode set to {}", self.is_debug_mode),
                        );
                    }
                    ui.menu_button("gcode serial count", |ui| {
                        if ui
                            .selectable_value(
                            &mut self.session_config.gcode_serial_count_mode,
                            GcodeSerialCountMode::CharCount,
                            "char count",
                        )
                        .changed()
                        {
                            self.append_telemetry_log("SETTINGS", "G-code serial mode set to char count");
                        }
                        if ui
                            .selectable_value(
                            &mut self.session_config.gcode_serial_count_mode,
                            GcodeSerialCountMode::RxLogs,
                            "RX logs",
                        )
                        .changed()
                        {
                            self.append_telemetry_log("SETTINGS", "G-code serial mode set to RX logs");
                        }
                    });
                    ui.label(format!(
                        "gcode serial count: {}",
                        self.session_config.gcode_serial_count_mode.label()
                    ));
                    ui.label(format!(
                        "No RX Prompt Exit: {}",
                        self.session_config.no_rx_prompt_exit
                    ));
                    ui.label(format!("Debug Mode: {}", self.is_debug_mode));
                    ui.separator();
                    if ui.button("Reset workspace view").clicked() {
                        self.reset_workspace_view();
                        ui.close_menu();
                    }
                });
                if self.is_debug_mode {
                    ui.menu_button("Debug", |ui| {
                        ui.label("Current status");
                        ui.separator();
                        ui.monospace(self.status.as_str());
                        ui.separator();
                        ui.menu_button("Serial", |ui| {
                            ui.monospace(format!("serial_connected: {}", self.serial_connected));
                            ui.monospace(format!("serial_address: {}", self.serial_address));
                            ui.monospace(format!("serial_baud: {}", self.serial_baud));
                            ui.monospace(format!("serial_log_lines: {}", self.serial_log.len()));
                        });
                        ui.menu_button("Print & RX", |ui| {
                            ui.monospace(format!("print_active: {}", self.print_active));
                            ui.monospace(format!("loop_active: {}", self.loop_active));
                            ui.monospace(format!("gcode_lines: {}", self.gcode_lines.len()));
                            ui.monospace(format!("gcode_progress_sent: {}", self.gcode_progress_sent));
                            ui.monospace(format!(
                                "gcode_progress_total: {}",
                                self.gcode_progress_total
                            ));
                            ui.monospace(format!("rx_buffer_used: {}", self.rx_buffer_used));
                            ui.monospace(format!(
                                "rx_buffer_capacity: {}",
                                self.rx_buffer_capacity
                            ));
                            ui.monospace(format!(
                                "laser_position_set: {}",
                                self.laser_position.is_some()
                            ));
                            ui.monospace(format!(
                                "session_mode: {}",
                                self.session_config.gcode_serial_count_mode.label()
                            ));
                        });
                        ui.menu_button("GUI", |ui| {
                            ui.monospace(format!("loop_selecting: {}", self.loop_selecting));
                            ui.monospace(format!(
                                "loop_selection_anchor_set: {}",
                                self.loop_selection_anchor.is_some()
                            ));
                            ui.monospace(format!(
                                "loop_rectangle_set: {}",
                                self.loop_rectangle.is_some()
                            ));
                            ui.monospace(format!(
                                "show_millimeter_grid: {}",
                                self.show_millimeter_grid
                            ));
                            ui.monospace(format!(
                                "show_workspace_grid: {}",
                                self.show_workspace_grid
                            ));
                            ui.monospace(format!(
                                "show_workspace_axes: {}",
                                self.show_workspace_axes
                            ));
                            ui.monospace(format!(
                                "gcode_polyline_points: {}",
                                self.gcode_polyline.len()
                            ));
                        });
                        ui.menu_button("Other", |ui| {
                            ui.monospace(format!(
                                "controller_reset_required: {}",
                                self.controller_reset_required
                            ));
                            ui.monospace(format!("jog_step_index: {}", self.jog_step_index));
                        });
                        ui.separator();
                        let telemetry_changed = ui
                            .checkbox(
                                &mut self.telemetry_enabled,
                                "Rust_Laser_GUI_Telemetry",
                            )
                            .changed();
                        if telemetry_changed {
                            if self.telemetry_enabled {
                                self.append_telemetry_log("TELEMETRY", "Telemetry Enabled");
                            } else {
                                self.telemetry_enabled = true;
                                self.append_telemetry_log("TELEMETRY", "Telemetry Disabled");
                                self.telemetry_enabled = false;
                            }
                        }
                    });
                }
            });
        });

        if self.rx_fallback_prompt_open {
            egui::Window::new("RX Status Fallback")
                .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
                .collapsible(false)
                .resizable(false)
                .show(ctx, |ui| {
                    ui.set_min_width(420.0);
                    ui.label(
                        "The controller is not reporting RX buffer status fields in `RX logs` mode.",
                    );
                    ui.add_space(8.0);
                    ui.label(
                        "All G-code lines were acknowledged locally. Choose Yes to keep tracking motion until the controller reports Idle.",
                    );
                    ui.add_space(4.0);
                    ui.label("Choose Continue to keep tracking motion until the controller reports Idle.");
                    ui.add_space(12.0);
                    ui.horizontal_centered(|ui| {
                        let button_size = egui::vec2(140.0, 44.0);

                        ui.add_space(85.0);
                        let continue_clicked = ui
                            .add_sized(
                                button_size,
                                egui::Button::new(egui::RichText::new("Continue").strong()),
                            )
                            .clicked();
                        let stop_clicked = ui
                            .add_sized(
                                button_size,
                                egui::Button::new(
                                    egui::RichText::new("Stop")
                                        .strong()
                                        .color(egui::Color32::WHITE),
                                )
                                .fill(egui::Color32::from_rgb(180, 40, 40)),
                            )
                            .clicked();

                        if continue_clicked {
                            self.confirm_rx_fallback_continue();
                        }
                        if stop_clicked {
                            self.reject_rx_fallback_continue();
                        }
                    });
                });
        }

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
                                let msg = self.serial_input.trim().to_string();
                                if !msg.is_empty() {
                                    self.append_telemetry_log(
                                        "SERIAL",
                                        &format!("Manual serial send requested: {msg}"),
                                    );
                                    match self
                                        .serial_worker
                                        .send(SerialCommand::SendLine(msg.to_string()))
                                    {
                                        Ok(()) => {}
                                        Err(err) => {
                                            self.serial_log.push(format!("Send failed: {err}"));
                                            self.append_telemetry_log(
                                                "ERROR",
                                                &format!("Manual serial send failed: {err}"),
                                            );
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
                            self.append_telemetry_log("SERIAL", "Disconnect requested");
                            if let Err(err) = self.serial_worker.send(SerialCommand::Disconnect) {
                                self.serial_log.push(format!("Disconnect failed: {err}"));
                                self.append_telemetry_log(
                                    "ERROR",
                                    &format!("Disconnect request failed: {err}"),
                                );
                            }
                        } else {
                            self.append_telemetry_log(
                                "SERIAL",
                                &format!(
                                    "Connect requested: address={}, baud={}",
                                    self.serial_address, self.serial_baud
                                ),
                            );
                            if let Err(err) = self.serial_worker.send(SerialCommand::Connect {
                                address: self.serial_address.clone(),
                                baud: self.serial_baud,
                            }) {
                                self.serial_log.push(format!("Connect failed: {err}"));
                                self.append_telemetry_log(
                                    "ERROR",
                                    &format!("Connect request failed: {err}"),
                                );
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

                        ui.add_space(8.0);
                        ui.label("Controller RX fill");
                        let rx_fill = if self.rx_buffer_capacity > 0 {
                            self.rx_buffer_used as f32 / self.rx_buffer_capacity as f32
                        } else {
                            0.0
                        };
                        let rx_fill_percent = rx_fill * 100.0;
                        ui.add(
                            egui::ProgressBar::new(rx_fill)
                                .desired_width(ui.available_width())
                                .text(format!(
                                    "{} / {} ({rx_fill_percent:.0}%)",
                                    self.rx_buffer_used, self.rx_buffer_capacity
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
                    if ui
                        .add(
                        egui::Slider::new(&mut self.jog_step_index, 0..=3)
                            .step_by(1.0)
                            .show_value(false),
                    )
                    .changed()
                    {
                        self.append_telemetry_log(
                            "GUI",
                            &format!("Jog step index changed to {}", self.jog_step_index),
                        );
                    }
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
                if ui
                    .add(
                    egui::Slider::new(&mut self.jog_feed, 50.0..=3000.0)
                        .text("Feed (mm/min)")
                        .step_by(10.0),
                )
                .changed()
                {
                    self.append_telemetry_log(
                        "GUI",
                        &format!("Jog feed changed to {:.0}", self.jog_feed),
                    );
                }
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
                                        } else if button_id == 4 {
                                            pad_left_panel_button_label(if self.loop_active
                                                || self.loop_selecting
                                            {
                                                "Stop Loop"
                                            } else {
                                                "Loop"
                                            })
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
                                        } else if button_id == 4
                                            && (self.loop_active || self.loop_selecting)
                                        {
                                            ui.add_sized(
                                                matrix_button_size,
                                                egui::Button::new(
                                                    egui::RichText::new(label.as_str())
                                                        .strong()
                                                        .color(egui::Color32::WHITE),
                                                )
                                                .fill(egui::Color32::from_rgb(180, 40, 40)),
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
                                                self.append_telemetry_log("ACTION", "Home requested");
                                                if let Err(err) =
                                                    self.serial_worker.send(SerialCommand::Home {
                                                        feed: self.jog_feed,
                                                    })
                                                {
                                                    self.serial_log
                                                        .push(format!("Home failed: {err}"));
                                                    self.append_telemetry_log(
                                                        "ERROR",
                                                        &format!("Home command failed: {err}"),
                                                    );
                                                }
                                            } else if button_id == 2 {
                                                if self.controller_reset_required {
                                                    self.flush_controller();
                                                }
                                            } else if button_id == 3 {
                                                if self.controller_reset_required {
                                                    self.continue_program();
                                                }
                                            } else if button_id == 4 {
                                                self.toggle_loop_mode();
                                            } else {
                                                self.append_telemetry_log(
                                                    "ACTION",
                                                    &format!("Placeholder matrix button {button_id} pressed"),
                                                );
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
                        self.append_telemetry_log("FILE", "Open file dialog requested");
                        if let Some(path) = FileDialog::new()
                            .add_filter("G-code", &["gcode", "nc"])
                            .pick_file()
                        {
                            self.file_path = path.display().to_string();
                            self.append_telemetry_log(
                                "FILE",
                                &format!("File selected from dialog: {}", self.file_path),
                            );
                            self.load_file();
                        }
                    }
                    if ui.button("Load").clicked() {
                        self.append_telemetry_log("FILE", "Load button pressed");
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
            let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click_and_drag());
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

            if response.clicked() {
                if let Some(pointer) = response.interact_pointer_pos() {
                    let world = clamp_world_to_workspace(
                        screen_to_world(pointer, plot, self.canvas_center, scale),
                        x_max,
                        y_max,
                    );
                    self.handle_loop_selection_click(world);
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

            let preview_rectangle = if self.loop_selecting {
                if let Some(anchor) = self.loop_selection_anchor {
                    response.hover_pos().map(|pointer| {
                        [
                            anchor,
                            clamp_world_to_workspace(
                                screen_to_world(pointer, plot, self.canvas_center, scale),
                                x_max,
                                y_max,
                            ),
                        ]
                    })
                } else {
                    self.loop_rectangle
                }
            } else {
                self.loop_rectangle
            };

            if let Some(rectangle) = preview_rectangle {
                draw_loop_rectangle(
                    &clipped,
                    rectangle,
                    plot,
                    self.canvas_center,
                    scale,
                    self.loop_active,
                );
            }

            let show_laser_marker = (self.session_config.gcode_serial_count_mode
                == GcodeSerialCountMode::RxLogs
                && self.is_printing())
                || self.loop_active;
            if show_laser_marker {
                if let Some(position) = self.laser_position {
                    let p = world_to_screen(position, plot, self.canvas_center, scale);
                    let marker_color = if self.loop_active {
                        egui::Color32::from_rgb(72, 210, 96)
                    } else {
                        egui::Color32::from_rgb(255, 120, 80)
                    };
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

fn clamp_world_to_workspace(point: [f32; 2], x_max: f32, y_max: f32) -> [f32; 2] {
    [point[0].clamp(0.0, x_max), point[1].clamp(0.0, y_max)]
}

fn draw_loop_rectangle(
    painter: &egui::Painter,
    rectangle: [[f32; 2]; 2],
    plot: egui::Rect,
    canvas_center: [f32; 2],
    scale: f32,
    active: bool,
) {
    let [a, b] = rectangle;
    let min_x = a[0].min(b[0]);
    let max_x = a[0].max(b[0]);
    let min_y = a[1].min(b[1]);
    let max_y = a[1].max(b[1]);

    let top_left = world_to_screen([min_x, max_y], plot, canvas_center, scale);
    let bottom_right = world_to_screen([max_x, min_y], plot, canvas_center, scale);
    let stroke = if active {
        egui::Stroke::new(2.5, egui::Color32::from_rgb(255, 220, 64))
    } else {
        egui::Stroke::new(1.8, egui::Color32::from_rgb(255, 210, 72))
    };
    painter.rect_stroke(
        egui::Rect::from_two_pos(top_left, bottom_right),
        0.0,
        stroke,
    );
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
