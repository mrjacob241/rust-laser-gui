use super::serial_bridge::SerialBridge;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

const GRBL_RX_LIMIT: usize = 127;
const STATUS_GUI_UPDATE_MS: u64 = 500;
const STATUS_QUERY_MS: u64 = 200;
const STREAM_POLL_MS: u64 = 20;
const STREAM_STALL_TIMEOUT_SECS: u64 = 30;
const LOOP_STREAK_LOOPS: usize = 2;
const RX_EMPTY_FINISH_MS: u64 = 1000;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum GcodeSerialCountMode {
    CharCount,
    RxLogs,
}

impl GcodeSerialCountMode {
    pub fn label(self) -> &'static str {
        match self {
            Self::CharCount => "char count",
            Self::RxLogs => "RX logs",
        }
    }
}

#[derive(Clone, Copy)]
pub struct SessionConfig {
    pub gcode_serial_count_mode: GcodeSerialCountMode,
    pub no_rx_prompt_exit: bool,
}

pub enum SerialCommand {
    Connect {
        address: String,
        baud: u32,
    },
    Disconnect,
    SendLine(String),
    SendLoadedGcode {
        lines: Vec<String>,
        session_config: SessionConfig,
    },
    FlushController,
    ContinueProgram,
    ResetToOrigin,
    JogMove {
        dx: f32,
        dy: f32,
        step: f32,
        feed: f32,
    },
    JogStop,
    Home {
        feed: f32,
    },
    LoopRectangle {
        corners: [[f32; 2]; 2],
        feed: f32,
    },
}

pub enum SerialEvent {
    Log(String),
    Connected(bool),
    PrintStarted,
    PrintFinished,
    RxFallbackContinuePrompt,
    GcodeProgress {
        sent: usize,
        total: usize,
    },
    RxBufferFill {
        used: usize,
        capacity: usize,
    },
    LoopStarted,
    LoopStopped,
    PrintAborted,
    LaserPosition {
        position: [f32; 2],
    },
}

pub struct SerialWorker {
    tx: Sender<SerialCommand>,
    rx: Receiver<SerialEvent>,
    stop_requested: Arc<AtomicBool>,
    stop_reset_requested: Arc<AtomicBool>,
    fallback_continue_requested: Arc<AtomicBool>,
}

impl SerialWorker {
    pub fn spawn() -> Self {
        let (tx_cmd, rx_cmd) = mpsc::channel::<SerialCommand>();
        let (tx_evt, rx_evt) = mpsc::channel::<SerialEvent>();
        let stop_requested = Arc::new(AtomicBool::new(false));
        let worker_stop_requested = Arc::clone(&stop_requested);
        let stop_reset_requested = Arc::new(AtomicBool::new(false));
        let worker_stop_reset_requested = Arc::clone(&stop_reset_requested);
        let fallback_continue_requested = Arc::new(AtomicBool::new(false));
        let worker_fallback_continue_requested = Arc::clone(&fallback_continue_requested);

        thread::spawn(move || {
            worker_loop(
                rx_cmd,
                tx_evt,
                worker_stop_requested,
                worker_stop_reset_requested,
                worker_fallback_continue_requested,
            )
        });

        Self {
            tx: tx_cmd,
            rx: rx_evt,
            stop_requested,
            stop_reset_requested,
            fallback_continue_requested,
        }
    }

    pub fn send(&self, cmd: SerialCommand) -> Result<(), String> {
        if matches!(
            cmd,
            SerialCommand::SendLoadedGcode { .. } | SerialCommand::LoopRectangle { .. }
        ) {
            self.stop_requested.store(false, Ordering::SeqCst);
            self.stop_reset_requested.store(false, Ordering::SeqCst);
            self.fallback_continue_requested
                .store(false, Ordering::SeqCst);
        }
        self.tx
            .send(cmd)
            .map_err(|_| "Serial worker is not available".to_string())
    }

    pub fn request_stop(&self) {
        self.stop_requested.store(true, Ordering::SeqCst);
        self.stop_reset_requested.store(false, Ordering::SeqCst);
    }

    pub fn request_stop_with_reset(&self) {
        self.stop_reset_requested.store(true, Ordering::SeqCst);
        self.stop_requested.store(true, Ordering::SeqCst);
    }

    pub fn confirm_fallback_continue(&self) {
        self.fallback_continue_requested
            .store(true, Ordering::SeqCst);
    }

    pub fn try_recv(&self) -> Option<SerialEvent> {
        self.rx.try_recv().ok()
    }
}

fn worker_loop(
    rx_cmd: Receiver<SerialCommand>,
    tx_evt: Sender<SerialEvent>,
    stop_requested: Arc<AtomicBool>,
    stop_reset_requested: Arc<AtomicBool>,
    fallback_continue_requested: Arc<AtomicBool>,
) {
    let mut bridge = SerialBridge::new();
    let mut track_position_until_idle = false;
    let mut last_resume_status_query = Instant::now() - Duration::from_millis(STATUS_QUERY_MS);

    loop {
        match rx_cmd.recv_timeout(Duration::from_millis(20)) {
            Ok(cmd) => handle_command(
                cmd,
                &mut bridge,
                &tx_evt,
                &stop_requested,
                &stop_reset_requested,
                &fallback_continue_requested,
                &mut track_position_until_idle,
            ),
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }

        if bridge.is_connected()
            && track_position_until_idle
            && Instant::now().duration_since(last_resume_status_query)
                >= Duration::from_millis(STATUS_QUERY_MS)
        {
            match bridge.send_status_query() {
                Ok(()) => {
                    last_resume_status_query = Instant::now();
                }
                Err(err) => {
                    let _ = tx_evt.send(SerialEvent::Log(format!("Status query failed: {err}")));
                    track_position_until_idle = false;
                }
            }
        }

        if bridge.is_connected() {
            let poll_wait_ms = if track_position_until_idle {
                STREAM_POLL_MS
            } else {
                2
            };
            match bridge.poll_reply(poll_wait_ms) {
                Ok(Some(reply)) => {
                    if track_position_until_idle {
                        handle_resume_tracking_reply(
                            &tx_evt,
                            &reply,
                            &mut track_position_until_idle,
                        );
                    } else {
                        emit_lines(&tx_evt, &reply);
                    }
                }
                Ok(None) => {}
                Err(err) => {
                    let _ = tx_evt.send(SerialEvent::Log(format!("Serial error: {err}")));
                    bridge.disconnect();
                    track_position_until_idle = false;
                    let _ = tx_evt.send(SerialEvent::RxBufferFill {
                        used: 0,
                        capacity: GRBL_RX_LIMIT,
                    });
                    let _ = tx_evt.send(SerialEvent::Connected(false));
                }
            }
        }
    }
}

fn handle_command(
    cmd: SerialCommand,
    bridge: &mut SerialBridge,
    tx_evt: &Sender<SerialEvent>,
    stop_requested: &Arc<AtomicBool>,
    stop_reset_requested: &Arc<AtomicBool>,
    fallback_continue_requested: &Arc<AtomicBool>,
    track_position_until_idle: &mut bool,
) {
    match cmd {
        SerialCommand::Connect { address, baud } => {
            *track_position_until_idle = false;
            if bridge.is_connected() {
                bridge.disconnect();
                let _ = tx_evt.send(SerialEvent::RxBufferFill {
                    used: 0,
                    capacity: GRBL_RX_LIMIT,
                });
                let _ = tx_evt.send(SerialEvent::Connected(false));
            }
            match bridge.connect(&address, baud) {
                Ok(()) => {
                    let _ = tx_evt.send(SerialEvent::Log(format!(
                        "Connected to {address} @ {baud} baud."
                    )));
                    let _ = tx_evt.send(SerialEvent::Connected(true));
                }
                Err(err) => {
                    let _ = tx_evt.send(SerialEvent::Log(format!("Connect failed: {err}")));
                    let _ = tx_evt.send(SerialEvent::Connected(false));
                }
            }
        }
        SerialCommand::Disconnect => {
            *track_position_until_idle = false;
            bridge.disconnect();
            let _ = tx_evt.send(SerialEvent::RxBufferFill {
                used: 0,
                capacity: GRBL_RX_LIMIT,
            });
            let _ = tx_evt.send(SerialEvent::Log("Disconnected.".to_string()));
            let _ = tx_evt.send(SerialEvent::Connected(false));
        }
        SerialCommand::SendLine(line) => match bridge.send_line(&line) {
            Ok(()) => {
                let _ = tx_evt.send(SerialEvent::Log(format!("TX> {line}")));
            }
            Err(err) => {
                let _ = tx_evt.send(SerialEvent::Log(format!("Send failed: {err}")));
            }
        },
        SerialCommand::SendLoadedGcode {
            lines,
            session_config,
        } => match session_config.gcode_serial_count_mode {
            GcodeSerialCountMode::CharCount => {
                send_loaded_gcode_worker(
                    bridge,
                    tx_evt,
                    &lines,
                    stop_requested,
                    stop_reset_requested,
                );
            }
            GcodeSerialCountMode::RxLogs => {
                send_loaded_gcode_worker_rx(
                    bridge,
                    tx_evt,
                    &lines,
                    session_config,
                    stop_requested,
                    stop_reset_requested,
                    fallback_continue_requested,
                );
            }
        },
        SerialCommand::FlushController => {
            let result = bridge.send_soft_reset();
            match result {
                Ok(()) => {
                    stop_requested.store(false, Ordering::SeqCst);
                    stop_reset_requested.store(false, Ordering::SeqCst);
                    *track_position_until_idle = false;
                    let _ = tx_evt.send(SerialEvent::RxBufferFill {
                        used: 0,
                        capacity: GRBL_RX_LIMIT,
                    });
                    let _ = tx_evt.send(SerialEvent::Log(
                        "TX> Ctrl-X (soft reset / flush controller state)".to_string(),
                    ));
                }
                Err(err) => {
                    let _ = tx_evt.send(SerialEvent::Log(format!("Flush failed: {err}")));
                }
            }
        }
        SerialCommand::ContinueProgram => match bridge.send_cycle_start() {
            Ok(()) => {
                stop_requested.store(false, Ordering::SeqCst);
                stop_reset_requested.store(false, Ordering::SeqCst);
                *track_position_until_idle = true;
                let _ = tx_evt.send(SerialEvent::Log("TX> ~ (cycle start)".to_string()));
                let _ = tx_evt.send(SerialEvent::PrintStarted);
            }
            Err(err) => {
                let _ = tx_evt.send(SerialEvent::Log(format!("Continue failed: {err}")));
            }
        },
        SerialCommand::ResetToOrigin => {
            let result = bridge
                .send_line("G90")
                .and_then(|_| bridge.send_line("G0 X0 Y0"));
            match result {
                Ok(()) => {
                    let _ = tx_evt.send(SerialEvent::Log("TX> G90 | G0 X0 Y0".to_string()));
                }
                Err(err) => {
                    let _ = tx_evt.send(SerialEvent::Log(format!("Reset failed: {err}")));
                }
            }
        }
        SerialCommand::JogMove { dx, dy, step, feed } => {
            let move_x = dx * step;
            let move_y = dy * step;
            let mut command = String::from("G1");
            if move_x.abs() > f32::EPSILON {
                command.push_str(&format!(" X{move_x:.3}"));
            }
            if move_y.abs() > f32::EPSILON {
                command.push_str(&format!(" Y{move_y:.3}"));
            }
            if command == "G1" {
                return;
            }
            command.push_str(&format!(" F{:.0}", feed.max(1.0)));
            let result = bridge
                .send_line("G91")
                .and_then(|_| bridge.send_line(&command))
                .and_then(|_| bridge.send_line("G90"));
            match result {
                Ok(()) => {
                    let _ = tx_evt.send(SerialEvent::Log(format!("TX> G91 | {command} | G90")));
                }
                Err(err) => {
                    let _ = tx_evt.send(SerialEvent::Log(format!("Jog failed: {err}")));
                }
            }
        }
        SerialCommand::JogStop => match bridge.send_feed_hold() {
            Ok(()) => {
                stop_requested.store(false, Ordering::SeqCst);
                stop_reset_requested.store(false, Ordering::SeqCst);
                let _ = tx_evt.send(SerialEvent::Log("TX> ! (feed hold)".to_string()));
                if let Err(err) = bridge.send_spindle_stop() {
                    let _ = tx_evt.send(SerialEvent::Log(format!(
                        "Laser-off request failed after stop: {err}"
                    )));
                } else {
                    let _ = tx_evt.send(SerialEvent::Log(
                        "TX> spindle/laser stop override".to_string(),
                    ));
                }
            }
            Err(err) => {
                let _ = tx_evt.send(SerialEvent::Log(format!("Stop failed: {err}")));
            }
        },
        SerialCommand::Home { feed } => {
            let home_cmd = format!("G1 X0 Y0 F{:.0}", feed.max(1.0));
            let result = bridge
                .send_line("G90")
                .and_then(|_| bridge.send_line(&home_cmd));
            match result {
                Ok(()) => {
                    let _ = tx_evt.send(SerialEvent::Log(format!("TX> G90 | {home_cmd}")));
                }
                Err(err) => {
                    let _ = tx_evt.send(SerialEvent::Log(format!("Home failed: {err}")));
                }
            }
        }
        SerialCommand::LoopRectangle { corners, feed } => {
            loop_rectangle_worker(bridge, tx_evt, corners, feed, stop_requested);
        }
    }
}

fn handle_resume_tracking_reply(
    tx_evt: &Sender<SerialEvent>,
    raw: &str,
    track_position_until_idle: &mut bool,
) {
    for raw_line in raw.lines() {
        let text = raw_line.trim();
        if text.is_empty() {
            continue;
        }

        let lower = text.to_ascii_lowercase();
        if lower == "ok" || lower.starts_with("error:") {
            continue;
        }

        if text.starts_with('<') {
            if let Some(report) = parse_grbl_status_report(text) {
                if let Some(position) = report.position {
                    let _ = tx_evt.send(SerialEvent::LaserPosition { position });
                }
                if report.machine_state == Some(GrblMachineState::Idle) {
                    *track_position_until_idle = false;
                    let _ = tx_evt.send(SerialEvent::PrintFinished);
                }
            }
            continue;
        }

        let _ = tx_evt.send(SerialEvent::Log(format!("RX> {text}")));
    }
}

fn send_loaded_gcode_worker(
    bridge: &mut SerialBridge,
    tx_evt: &Sender<SerialEvent>,
    lines: &[String],
    stop_requested: &Arc<AtomicBool>,
    stop_reset_requested: &Arc<AtomicBool>,
) {
    if lines.is_empty() {
        let _ = tx_evt.send(SerialEvent::GcodeProgress { sent: 0, total: 0 });
        let _ = tx_evt.send(SerialEvent::Log("No G-code loaded.".to_string()));
        return;
    }

    let total = lines.len();
    let _ = tx_evt.send(SerialEvent::PrintStarted);
    let _ = tx_evt.send(SerialEvent::GcodeProgress { sent: 0, total });
    let _ = tx_evt.send(SerialEvent::RxBufferFill {
        used: 0,
        capacity: GRBL_RX_LIMIT,
    });

    let mut sent = 0_u64;
    let mut acked = 0_u64;
    let mut next_index = 0_usize;
    let mut rx_used = 0_usize;
    let mut in_flight: VecDeque<(usize, usize, String)> = VecDeque::new(); // (bytes, line_no, text)
    let mut last_progress = Instant::now();
    let mut last_status_emit = Instant::now() - Duration::from_millis(STATUS_GUI_UPDATE_MS);

    loop {
        if stop_requested.load(Ordering::SeqCst) {
            stop_stream(bridge, tx_evt, stop_requested, stop_reset_requested);
            return;
        }

        // Character-counting: keep RX near full without overflow.
        while next_index < lines.len() {
            if stop_requested.load(Ordering::SeqCst) {
                stop_stream(bridge, tx_evt, stop_requested, stop_reset_requested);
                return;
            }
            let line = lines[next_index].clone();
            let bytes = line.len() + 1; // include newline
            if bytes > GRBL_RX_LIMIT {
                let _ = tx_evt.send(SerialEvent::Log(format!(
                    "Line {} too long for GRBL RX buffer ({} > {}).",
                    next_index + 1,
                    bytes,
                    GRBL_RX_LIMIT
                )));
                return;
            }
            if rx_used + bytes > GRBL_RX_LIMIT {
                break;
            }

            match bridge.send_line(&line) {
                Ok(()) => {
                    sent += 1;
                    rx_used += bytes;
                    in_flight.push_back((bytes, next_index + 1, line));
                    next_index += 1;
                    last_progress = Instant::now();
                    let _ = tx_evt.send(SerialEvent::GcodeProgress {
                        sent: next_index,
                        total,
                    });
                }
                Err(err) => {
                    let _ = tx_evt.send(SerialEvent::Log(format!("Send failed: {err}")));
                    return;
                }
            }
        }

        if next_index >= lines.len() && in_flight.is_empty() {
            break;
        }

        if Instant::now().duration_since(last_progress)
            > Duration::from_secs(STREAM_STALL_TIMEOUT_SECS)
        {
            let _ = tx_evt.send(SerialEvent::Log(
                "Stream stalled waiting for acknowledgements.".to_string(),
            ));
            return;
        }

        match bridge.poll_reply(STREAM_POLL_MS) {
            Ok(Some(reply)) => {
                for raw_line in reply.lines() {
                    let text = raw_line.trim();
                    if text.is_empty() {
                        continue;
                    }

                    let lower = text.to_ascii_lowercase();
                    if lower == "ok" || lower.starts_with("error:") {
                        if let Some((bytes, line_no, line_text)) = in_flight.pop_front() {
                            rx_used = rx_used.saturating_sub(bytes);
                            acked += 1;
                            last_progress = Instant::now();

                            if lower.starts_with("error:") {
                                let _ = tx_evt.send(SerialEvent::Log(format!(
                                    "Device error on line {line_no}: {text} ({line_text})"
                                )));
                                return;
                            }
                        } else if lower.starts_with("error:") {
                            let _ = tx_evt.send(SerialEvent::Log(format!(
                                "Device error with empty in-flight queue: {text}"
                            )));
                            return;
                        }
                        continue;
                    }

                    if text.starts_with('<') {
                        let now = Instant::now();
                        if now.duration_since(last_status_emit)
                            >= Duration::from_millis(STATUS_GUI_UPDATE_MS)
                        {
                            let _ = tx_evt.send(SerialEvent::Log(format!("STATUS> {text}")));
                            last_status_emit = now;
                        }
                        continue;
                    }

                    let _ = tx_evt.send(SerialEvent::Log(format!("RX> {text}")));
                }
            }
            Ok(None) => {}
            Err(err) => {
                let _ = tx_evt.send(SerialEvent::Log(format!("Send failed: {err}")));
                return;
            }
        }
    }

    let _ = tx_evt.send(SerialEvent::Log(format!(
        "Stream completed: sent {sent}, acked {acked}, rx_used {rx_used}/{GRBL_RX_LIMIT}."
    )));
    let _ = tx_evt.send(SerialEvent::PrintFinished);
    let _ = tx_evt.send(SerialEvent::Log(
        "[Printing G-Code] completed successfully".to_string(),
    ));
}

fn send_loaded_gcode_worker_rx(
    bridge: &mut SerialBridge,
    tx_evt: &Sender<SerialEvent>,
    lines: &[String],
    session_config: SessionConfig,
    stop_requested: &Arc<AtomicBool>,
    stop_reset_requested: &Arc<AtomicBool>,
    fallback_continue_requested: &Arc<AtomicBool>,
) {
    if lines.is_empty() {
        let _ = tx_evt.send(SerialEvent::GcodeProgress { sent: 0, total: 0 });
        let _ = tx_evt.send(SerialEvent::Log("No G-code loaded.".to_string()));
        return;
    }

    let total = lines.len();
    let _ = tx_evt.send(SerialEvent::PrintStarted);
    let _ = tx_evt.send(SerialEvent::GcodeProgress { sent: 0, total });

    let mut sent = 0_u64;
    let mut acked = 0_u64;
    let mut next_index = 0_usize;
    let mut rx_used = 0_usize;
    let mut in_flight: VecDeque<(usize, usize, String)> = VecDeque::new(); // (bytes, line_no, text)
    let mut last_progress = Instant::now();
    let mut rx_free_hint: Option<usize> = None;
    let mut reported_fallback = false;
    let mut waiting_for_rx_empty_after_send = false;
    let mut rx_empty_since: Option<Instant> = None;
    let mut rx_empty_wait_announced = false;
    let mut fallback_prompt_pending = false;
    let mut fallback_continue_tracking = false;
    let mut fallback_continue_idle_detected = false;
    let (status_tick_tx, status_tick_rx) = mpsc::channel::<()>();
    let (status_stop_tx, status_stop_rx) = mpsc::channel::<()>();
    let status_thread = thread::spawn(move || {
        loop {
            match status_stop_rx.recv_timeout(Duration::from_millis(STATUS_QUERY_MS)) {
                Ok(()) | Err(mpsc::RecvTimeoutError::Disconnected) => break,
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    if status_tick_tx.send(()).is_err() {
                        break;
                    }
                }
            }
        }
    });

    loop {
        if stop_requested.load(Ordering::SeqCst) {
            let _ = status_stop_tx.send(());
            let _ = status_thread.join();
            stop_stream(bridge, tx_evt, stop_requested, stop_reset_requested);
            return;
        }

        while status_tick_rx.try_recv().is_ok() {
            if let Err(err) = bridge.send_status_query() {
                let _ = status_stop_tx.send(());
                let _ = status_thread.join();
                let _ = tx_evt.send(SerialEvent::Log(format!("Status query failed: {err}")));
                return;
            }
        }

        while next_index < lines.len() {
            if stop_requested.load(Ordering::SeqCst) {
                let _ = status_stop_tx.send(());
                let _ = status_thread.join();
                stop_stream(bridge, tx_evt, stop_requested, stop_reset_requested);
                return;
            }
            let line = lines[next_index].clone();
            let bytes = line.len() + 1;
            if bytes > GRBL_RX_LIMIT {
                let _ = status_stop_tx.send(());
                let _ = status_thread.join();
                let _ = tx_evt.send(SerialEvent::Log(format!(
                    "Line {} too long for GRBL RX buffer ({} > {}).",
                    next_index + 1,
                    bytes,
                    GRBL_RX_LIMIT
                )));
                return;
            }

            let local_rx_free = GRBL_RX_LIMIT.saturating_sub(rx_used);
            let effective_rx_free = match rx_free_hint {
                Some(rx_free) => local_rx_free.min(rx_free),
                None => local_rx_free,
            };
            let can_send = bytes <= effective_rx_free;

            if !can_send {
                if rx_free_hint.is_none() && !reported_fallback {
                    let _ = tx_evt.send(SerialEvent::Log(
                        "STATUS> No RX buffer field detected yet, continuing with local character counting."
                            .to_string(),
                    ));
                    reported_fallback = true;
                }
                break;
            }

            match bridge.send_line(&line) {
                Ok(()) => {
                    sent += 1;
                    rx_used += bytes;
                    in_flight.push_back((bytes, next_index + 1, line));
                    next_index += 1;
                    last_progress = Instant::now();
                    let _ = tx_evt.send(SerialEvent::GcodeProgress {
                        sent: next_index,
                        total,
                    });
                    if let Some(rx_free) = rx_free_hint.as_mut() {
                        *rx_free = rx_free.saturating_sub(bytes);
                    }
                }
                Err(err) => {
                    let _ = status_stop_tx.send(());
                    let _ = status_thread.join();
                    let _ = tx_evt.send(SerialEvent::Log(format!("Send failed: {err}")));
                    return;
                }
            }
        }

        if next_index >= lines.len() {
            if rx_free_hint.is_some() {
                waiting_for_rx_empty_after_send = true;
                if rx_empty_since.is_some_and(|since| {
                    Instant::now().duration_since(since)
                        >= Duration::from_millis(RX_EMPTY_FINISH_MS)
                }) {
                    let _ = tx_evt.send(SerialEvent::Log(format!(
                        "STATUS> RX fill stayed at 0% for {} ms after 100% send; concluding print.",
                        RX_EMPTY_FINISH_MS
                    )));
                    break;
                }
            } else if !fallback_continue_tracking {
                if session_config.no_rx_prompt_exit {
                    let _ = tx_evt.send(SerialEvent::Log(
                        "STATUS> 100% send reached without RX buffer status fields; `No RX Prompt Exit` is enabled, concluding print immediately."
                            .to_string(),
                    ));
                    break;
                }
                if !fallback_prompt_pending {
                    fallback_prompt_pending = true;
                    let _ = tx_evt.send(SerialEvent::Log(
                        "STATUS> 100% send reached without RX buffer status fields; waiting for user confirmation to continue or stop."
                            .to_string(),
                    ));
                    let _ = tx_evt.send(SerialEvent::RxFallbackContinuePrompt);
                }

                if fallback_continue_requested.swap(false, Ordering::SeqCst) {
                    fallback_prompt_pending = false;
                    fallback_continue_tracking = true;
                    let _ = tx_evt.send(SerialEvent::Log(
                        "STATUS> User chose Continue; tracking controller status until Idle because RX buffer fields are unavailable."
                            .to_string(),
                    ));
                }
            }
        }

        if Instant::now().duration_since(last_progress)
            > Duration::from_secs(STREAM_STALL_TIMEOUT_SECS)
        {
            let _ = status_stop_tx.send(());
            let _ = status_thread.join();
            let _ = tx_evt.send(SerialEvent::Log(
                "RX-status stream stalled waiting for acknowledgements.".to_string(),
            ));
            return;
        }

        match bridge.poll_reply(STREAM_POLL_MS) {
            Ok(Some(reply)) => {
                for raw_line in reply.lines() {
                    let text = raw_line.trim();
                    if text.is_empty() {
                        continue;
                    }

                    let lower = text.to_ascii_lowercase();
                    if lower == "ok" || lower.starts_with("error:") {
                        if let Some((bytes, line_no, line_text)) = in_flight.pop_front() {
                            acked += 1;
                            rx_used = rx_used.saturating_sub(bytes);
                            last_progress = Instant::now();
                            if let Some(rx_free) = rx_free_hint.as_mut() {
                                *rx_free = (*rx_free + bytes).min(GRBL_RX_LIMIT);
                            }

                            if lower.starts_with("error:") {
                                let _ = status_stop_tx.send(());
                                let _ = status_thread.join();
                                let _ = tx_evt.send(SerialEvent::Log(format!(
                                    "Device error on line {line_no}: {text} ({line_text})"
                                )));
                                return;
                            }
                        } else if lower.starts_with("error:") {
                            let _ = status_stop_tx.send(());
                            let _ = status_thread.join();
                            let _ = tx_evt.send(SerialEvent::Log(format!(
                                "Device error with empty in-flight queue: {text}"
                            )));
                            return;
                        }
                        continue;
                    }

                    if text.starts_with('<') {
                        if let Some(report) = parse_grbl_status_report(text) {
                            last_progress = Instant::now();
                            if let Some(rx_free) = report.rx_free {
                                let rx_free = rx_free.min(GRBL_RX_LIMIT);
                                rx_free_hint = Some(rx_free);
                                rx_used = GRBL_RX_LIMIT.saturating_sub(rx_free);
                                let _ = tx_evt.send(SerialEvent::RxBufferFill {
                                    used: rx_used,
                                    capacity: GRBL_RX_LIMIT,
                                });
                                if waiting_for_rx_empty_after_send {
                                    if rx_used == 0 {
                                        if rx_empty_since.is_none() {
                                            rx_empty_since = Some(Instant::now());
                                            if !rx_empty_wait_announced {
                                                let _ = tx_evt.send(SerialEvent::Log(format!(
                                                    "STATUS> G-code send is 100% and RX fill is 0%; waiting {} ms before concluding print.",
                                                    RX_EMPTY_FINISH_MS
                                                )));
                                                rx_empty_wait_announced = true;
                                            }
                                        }
                                    } else {
                                        rx_empty_since = None;
                                        rx_empty_wait_announced = false;
                                    }
                                }
                            }
                            if let Some(position) = report.position {
                                let _ = tx_evt.send(SerialEvent::LaserPosition { position });
                            }
                            if fallback_continue_tracking
                                && next_index >= lines.len()
                                && report.machine_state == Some(GrblMachineState::Idle)
                            {
                                let _ = tx_evt.send(SerialEvent::Log(
                                    "STATUS> Controller reported Idle after user-approved fallback continue; concluding print."
                                        .to_string(),
                                ));
                                fallback_continue_idle_detected = true;
                                break;
                            }
                        }
                        continue;
                    }

                    let _ = tx_evt.send(SerialEvent::Log(format!("RX> {text}")));
                }
            }
            Ok(None) => {}
            Err(err) => {
                let _ = status_stop_tx.send(());
                let _ = status_thread.join();
                let _ = tx_evt.send(SerialEvent::Log(format!("Send failed: {err}")));
                return;
            }
        }

        if fallback_continue_idle_detected {
            break;
        }

    }

    let _ = status_stop_tx.send(());
    let _ = status_thread.join();
    let mode = if reported_fallback {
        "rx-status hybrid with local character counting"
    } else {
        "rx-status guided"
    };
    let _ = tx_evt.send(SerialEvent::Log(format!(
        "Stream completed ({mode}): sent {sent}, acked {acked}, rx_used {rx_used}/{GRBL_RX_LIMIT}, rx_free_hint {:?}.",
        rx_free_hint
    )));
    let _ = tx_evt.send(SerialEvent::RxBufferFill {
        used: 0,
        capacity: GRBL_RX_LIMIT,
    });
    let _ = tx_evt.send(SerialEvent::PrintFinished);
    let _ = tx_evt.send(SerialEvent::Log(
        "[Printing G-Code] completed successfully".to_string(),
    ));
}

fn loop_rectangle_worker(
    bridge: &mut SerialBridge,
    tx_evt: &Sender<SerialEvent>,
    corners: [[f32; 2]; 2],
    feed: f32,
    stop_requested: &Arc<AtomicBool>,
) {
    let [a, c] = corners;
    let min_x = a[0].min(c[0]);
    let max_x = a[0].max(c[0]);
    let min_y = a[1].min(c[1]);
    let max_y = a[1].max(c[1]);

    if (max_x - min_x).abs() <= f32::EPSILON || (max_y - min_y).abs() <= f32::EPSILON {
        let _ = tx_evt.send(SerialEvent::Log(
            "Loop rectangle requires two diagonal corners with non-zero width and height."
                .to_string(),
        ));
        let _ = tx_evt.send(SerialEvent::LoopStopped);
        return;
    }

    let loop_feed = feed.max(1.0);
    let vertices = [
        [min_x, min_y],
        [max_x, min_y],
        [max_x, max_y],
        [min_x, max_y],
    ];
    let loop_lines = [
        format!("G1 X{:.3} Y{:.3} F{:.0}", vertices[1][0], vertices[1][1], loop_feed),
        format!("G1 X{:.3} Y{:.3} F{:.0}", vertices[2][0], vertices[2][1], loop_feed),
        format!("G1 X{:.3} Y{:.3} F{:.0}", vertices[3][0], vertices[3][1], loop_feed),
        format!("G1 X{:.3} Y{:.3} F{:.0}", vertices[0][0], vertices[0][1], loop_feed),
    ];
    let loop_bytes: usize = loop_lines.iter().map(|line| line.len() + 1).sum();
    let target_rx_bytes = (loop_bytes * LOOP_STREAK_LOOPS).min(GRBL_RX_LIMIT);

    let _ = tx_evt.send(SerialEvent::LoopStarted);
    let _ = tx_evt.send(SerialEvent::Log(format!(
        "Loop rectangle started: ({min_x:.2}, {min_y:.2}) -> ({max_x:.2}, {max_y:.2}) @ F{loop_feed:.0} with laser off, target queue {LOOP_STREAK_LOOPS} loop(s)."
    )));

    if let Err(err) = bridge.send_spindle_stop() {
        let _ = tx_evt.send(SerialEvent::Log(format!(
            "Laser-off request failed before loop start: {err}"
        )));
    }

    if let Err(err) = send_blocking_line(bridge, tx_evt, "G90") {
        let _ = tx_evt.send(SerialEvent::Log(format!("Loop setup failed: {err}")));
        let _ = tx_evt.send(SerialEvent::LoopStopped);
        return;
    }

    let move_to_start = format!("G1 X{:.3} Y{:.3} F{:.0}", vertices[0][0], vertices[0][1], loop_feed);
    if let Err(err) = send_blocking_line(bridge, tx_evt, &move_to_start) {
        let _ = tx_evt.send(SerialEvent::Log(format!("Loop start move failed: {err}")));
        let _ = tx_evt.send(SerialEvent::LoopStopped);
        return;
    }

    let (status_tick_tx, status_tick_rx) = mpsc::channel::<()>();
    let (status_stop_tx, status_stop_rx) = mpsc::channel::<()>();
    let status_thread = thread::spawn(move || {
        loop {
            match status_stop_rx.recv_timeout(Duration::from_millis(STATUS_QUERY_MS)) {
                Ok(()) | Err(mpsc::RecvTimeoutError::Disconnected) => break,
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    if status_tick_tx.send(()).is_err() {
                        break;
                    }
                }
            }
        }
    });

    let mut rx_used = 0_usize;
    let mut rx_free_hint: Option<usize> = None;
    let mut next_index = 0_usize;
    let mut in_flight: VecDeque<usize> = VecDeque::new();
    let mut last_activity = Instant::now();

    'looping: loop {
        if stop_requested.load(Ordering::SeqCst) {
            break;
        }

        while status_tick_rx.try_recv().is_ok() {
            if let Err(err) = bridge.send_status_query() {
                let _ = tx_evt.send(SerialEvent::Log(format!(
                    "Loop status query failed: {err}"
                )));
                break 'looping;
            }
        }

        loop {
            if stop_requested.load(Ordering::SeqCst) {
                break 'looping;
            }

            let line = &loop_lines[next_index];
            let bytes = line.len() + 1;
            let local_rx_free = GRBL_RX_LIMIT.saturating_sub(rx_used);
            let effective_rx_free = match rx_free_hint {
                Some(rx_free) => local_rx_free.min(rx_free),
                None => local_rx_free,
            };

            if rx_used + bytes > target_rx_bytes || bytes > effective_rx_free {
                break;
            }

            match bridge.send_line(line) {
                Ok(()) => {
                    rx_used += bytes;
                    in_flight.push_back(bytes);
                    next_index = (next_index + 1) % loop_lines.len();
                    last_activity = Instant::now();
                    let _ = tx_evt.send(SerialEvent::RxBufferFill {
                        used: rx_used,
                        capacity: GRBL_RX_LIMIT,
                    });
                    if let Some(rx_free) = rx_free_hint.as_mut() {
                        *rx_free = rx_free.saturating_sub(bytes);
                    }
                }
                Err(err) => {
                    let _ = tx_evt.send(SerialEvent::Log(format!("Loop move failed: {err}")));
                    break 'looping;
                }
            }
        }

        if Instant::now().duration_since(last_activity)
            > Duration::from_secs(STREAM_STALL_TIMEOUT_SECS)
        {
            let _ = tx_evt.send(SerialEvent::Log(
                "Loop stream stalled waiting for controller progress.".to_string(),
            ));
            break;
        }

        match bridge.poll_reply(STREAM_POLL_MS) {
            Ok(Some(reply)) => {
                for raw_line in reply.lines() {
                    let text = raw_line.trim();
                    if text.is_empty() {
                        continue;
                    }

                    let lower = text.to_ascii_lowercase();
                    if lower == "ok" || lower.starts_with("error:") {
                        if let Some(bytes) = in_flight.pop_front() {
                            rx_used = rx_used.saturating_sub(bytes);
                            last_activity = Instant::now();
                            let _ = tx_evt.send(SerialEvent::RxBufferFill {
                                used: rx_used,
                                capacity: GRBL_RX_LIMIT,
                            });
                            if let Some(rx_free) = rx_free_hint.as_mut() {
                                *rx_free = (*rx_free + bytes).min(GRBL_RX_LIMIT);
                            }
                        }

                        if lower.starts_with("error:") {
                            let _ = tx_evt.send(SerialEvent::Log(format!(
                                "Loop device error: {text}"
                            )));
                            break 'looping;
                        }
                        continue;
                    }

                    if text.starts_with('<') {
                        if let Some(report) = parse_grbl_status_report(text) {
                            last_activity = Instant::now();
                            if let Some(rx_free) = report.rx_free {
                                let rx_free = rx_free.min(GRBL_RX_LIMIT);
                                rx_free_hint = Some(rx_free);
                                rx_used = GRBL_RX_LIMIT.saturating_sub(rx_free);
                                let _ = tx_evt.send(SerialEvent::RxBufferFill {
                                    used: rx_used,
                                    capacity: GRBL_RX_LIMIT,
                                });
                            }
                            if let Some(position) = report.position {
                                let _ = tx_evt.send(SerialEvent::LaserPosition { position });
                            }
                        }
                        continue;
                    }

                    let _ = tx_evt.send(SerialEvent::Log(format!("RX> {text}")));
                }
            }
            Ok(None) => {}
            Err(err) => {
                let _ = tx_evt.send(SerialEvent::Log(format!("Loop receive failed: {err}")));
                break;
            }
        }
    }

    let _ = status_stop_tx.send(());
    let _ = status_thread.join();
    if stop_requested.load(Ordering::SeqCst) {
        stop_loop_stream(bridge, tx_evt, stop_requested);
        let _ = tx_evt.send(SerialEvent::LoopStopped);
        let _ = tx_evt.send(SerialEvent::RxBufferFill {
            used: 0,
            capacity: GRBL_RX_LIMIT,
        });
        return;
    }
    if let Err(err) = bridge.send_spindle_stop() {
        let _ = tx_evt.send(SerialEvent::Log(format!(
            "Laser-off request failed after loop stop: {err}"
        )));
    }
    let _ = tx_evt.send(SerialEvent::LoopStopped);
    let _ = tx_evt.send(SerialEvent::RxBufferFill {
        used: 0,
        capacity: GRBL_RX_LIMIT,
    });
    let _ = tx_evt.send(SerialEvent::Log("Loop rectangle stopped.".to_string()));
    stop_requested.store(false, Ordering::SeqCst);
}

fn stop_loop_stream(
    bridge: &mut SerialBridge,
    tx_evt: &Sender<SerialEvent>,
    stop_requested: &Arc<AtomicBool>,
) {
    match bridge.send_feed_hold() {
        Ok(()) => {
            let _ = tx_evt.send(SerialEvent::Log("TX> ! (feed hold)".to_string()));
        }
        Err(err) => {
            let _ = tx_evt.send(SerialEvent::Log(format!("Stop failed: {err}")));
        }
    }
    match bridge.send_spindle_stop() {
        Ok(()) => {
            let _ = tx_evt.send(SerialEvent::Log(
                "TX> spindle/laser stop override".to_string(),
            ));
        }
        Err(err) => {
            let _ = tx_evt.send(SerialEvent::Log(format!(
                "Laser-off request failed after stop: {err}"
            )));
        }
    }
    match bridge.send_soft_reset() {
        Ok(()) => {
            let _ = tx_evt.send(SerialEvent::Log(
                "TX> Ctrl-X (soft reset after loop stop)".to_string(),
            ));
        }
        Err(err) => {
            let _ = tx_evt.send(SerialEvent::Log(format!(
                "Loop reset failed: {err}"
            )));
        }
    }
    let _ = tx_evt.send(SerialEvent::Log(
        "[Loop Rectangle] stopped by stop request and reset".to_string(),
    ));
    stop_requested.store(false, Ordering::SeqCst);
}

fn send_blocking_line(
    bridge: &mut SerialBridge,
    tx_evt: &Sender<SerialEvent>,
    line: &str,
) -> Result<(), String> {
    bridge.send_line(line)?;
    wait_for_serial_ack(bridge, tx_evt)
}

fn wait_for_serial_ack(bridge: &mut SerialBridge, tx_evt: &Sender<SerialEvent>) -> Result<(), String> {
    let start = Instant::now();
    loop {
        if Instant::now().duration_since(start) > Duration::from_secs(STREAM_STALL_TIMEOUT_SECS) {
            return Err("Timed out waiting for controller acknowledgement".to_string());
        }

        match bridge.poll_reply(STREAM_POLL_MS) {
            Ok(Some(reply)) => {
                for raw_line in reply.lines() {
                    let text = raw_line.trim();
                    if text.is_empty() {
                        continue;
                    }

                    let lower = text.to_ascii_lowercase();
                    if lower == "ok" {
                        return Ok(());
                    }
                    if lower.starts_with("error:") {
                        return Err(format!("Device error: {text}"));
                    }

                    if text.starts_with('<') {
                        if let Some(report) = parse_grbl_status_report(text) {
                            if let Some(position) = report.position {
                                let _ = tx_evt.send(SerialEvent::LaserPosition { position });
                            }
                        }
                        continue;
                    }

                    let _ = tx_evt.send(SerialEvent::Log(format!("RX> {text}")));
                }
            }
            Ok(None) => {}
            Err(err) => return Err(err),
        }
    }
}

fn stop_stream(
    bridge: &mut SerialBridge,
    tx_evt: &Sender<SerialEvent>,
    stop_requested: &Arc<AtomicBool>,
    stop_reset_requested: &Arc<AtomicBool>,
) {
    match bridge.send_feed_hold() {
        Ok(()) => {
            let _ = tx_evt.send(SerialEvent::Log("TX> ! (feed hold)".to_string()));
        }
        Err(err) => {
            let _ = tx_evt.send(SerialEvent::Log(format!("Stop failed: {err}")));
        }
    }
    match bridge.send_spindle_stop() {
        Ok(()) => {
            let _ = tx_evt.send(SerialEvent::Log(
                "TX> spindle/laser stop override".to_string(),
            ));
        }
        Err(err) => {
            let _ = tx_evt.send(SerialEvent::Log(format!(
                "Laser-off request failed after stop: {err}"
            )));
        }
    }
    if stop_reset_requested.load(Ordering::SeqCst) {
        match bridge.send_soft_reset() {
            Ok(()) => {
                let _ = tx_evt.send(SerialEvent::Log(
                    "TX> Ctrl-X (soft reset after popup stop)".to_string(),
                ));
            }
            Err(err) => {
                let _ = tx_evt.send(SerialEvent::Log(format!(
                    "Reset failed after stop: {err}"
                )));
            }
        }
    }
    let _ = tx_evt.send(SerialEvent::PrintAborted);
    let _ = tx_evt.send(SerialEvent::RxBufferFill {
        used: 0,
        capacity: GRBL_RX_LIMIT,
    });
    let abort_message = if stop_reset_requested.load(Ordering::SeqCst) {
        "[Printing G-Code] aborted by stop request and soft reset".to_string()
    } else {
        "[Printing G-Code] aborted by stop request".to_string()
    };
    let _ = tx_evt.send(SerialEvent::Log(abort_message));
    stop_requested.store(false, Ordering::SeqCst);
    stop_reset_requested.store(false, Ordering::SeqCst);
}

#[derive(Debug, Default, Clone, Copy)]
struct GrblStatusReport {
    machine_state: Option<GrblMachineState>,
    planner_free: Option<usize>,
    rx_free: Option<usize>,
    position: Option<[f32; 2]>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GrblMachineState {
    Idle,
    Other,
}

fn parse_grbl_status_report(line: &str) -> Option<GrblStatusReport> {
    let body = line.strip_prefix('<')?.strip_suffix('>')?;
    let mut report = GrblStatusReport::default();

    for (idx, field) in body.split('|').enumerate() {
        if idx == 0 {
            report.machine_state = Some(parse_machine_state(field));
            continue;
        }

        if let Some(rest) = field.strip_prefix("Bf:") {
            let mut parts = rest.split(',');
            report.planner_free = parts.next().and_then(parse_status_usize);
            report.rx_free = parts.next().and_then(parse_status_usize);
            continue;
        }

        if let Some(rest) = field.strip_prefix("Buf:") {
            report.planner_free = parse_status_usize(rest);
            continue;
        }

        if let Some(rest) = field.strip_prefix("RX:") {
            report.rx_free = parse_status_usize(rest);
            continue;
        }

        if let Some(rest) = field.strip_prefix("WPos:") {
            report.position = parse_status_position(rest);
            continue;
        }

        if report.position.is_none() {
            if let Some(rest) = field.strip_prefix("MPos:") {
                report.position = parse_status_position(rest);
            }
        }
    }

    Some(report)
}

fn parse_machine_state(value: &str) -> GrblMachineState {
    let state = value.split(':').next().unwrap_or("").trim();
    if state.eq_ignore_ascii_case("idle") {
        GrblMachineState::Idle
    } else {
        GrblMachineState::Other
    }
}

fn parse_status_usize(value: &str) -> Option<usize> {
    value.trim().parse::<usize>().ok()
}

fn parse_status_position(value: &str) -> Option<[f32; 2]> {
    let mut parts = value.split(',');
    let x = parts.next()?.trim().parse::<f32>().ok()?;
    let y = parts.next()?.trim().parse::<f32>().ok()?;
    Some([x, y])
}

fn emit_lines(tx_evt: &Sender<SerialEvent>, raw: &str) {
    for line in raw.lines() {
        let text = line.trim();
        if !text.is_empty() {
            let _ = tx_evt.send(SerialEvent::Log(format!("RX> {text}")));
        }
    }
}
