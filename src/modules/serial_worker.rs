use super::serial_bridge::SerialBridge;
use std::collections::VecDeque;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;
use std::time::{Duration, Instant};

const GRBL_RX_LIMIT: usize = 127;
const STATUS_GUI_UPDATE_MS: u64 = 500;
const STATUS_QUERY_MS: u64 = 200;
const STREAM_POLL_MS: u64 = 20;
const STREAM_STALL_TIMEOUT_SECS: u64 = 30;

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
}

pub enum SerialEvent {
    Log(String),
    Connected(bool),
    GcodeProgress {
        sent: usize,
        total: usize,
    },
    LaserPosition {
        position: [f32; 2],
    },
}

pub struct SerialWorker {
    tx: Sender<SerialCommand>,
    rx: Receiver<SerialEvent>,
}

impl SerialWorker {
    pub fn spawn() -> Self {
        let (tx_cmd, rx_cmd) = mpsc::channel::<SerialCommand>();
        let (tx_evt, rx_evt) = mpsc::channel::<SerialEvent>();

        thread::spawn(move || worker_loop(rx_cmd, tx_evt));

        Self {
            tx: tx_cmd,
            rx: rx_evt,
        }
    }

    pub fn send(&self, cmd: SerialCommand) -> Result<(), String> {
        self.tx
            .send(cmd)
            .map_err(|_| "Serial worker is not available".to_string())
    }

    pub fn try_recv(&self) -> Option<SerialEvent> {
        self.rx.try_recv().ok()
    }
}

fn worker_loop(rx_cmd: Receiver<SerialCommand>, tx_evt: Sender<SerialEvent>) {
    let mut bridge = SerialBridge::new();

    loop {
        match rx_cmd.recv_timeout(Duration::from_millis(20)) {
            Ok(cmd) => handle_command(cmd, &mut bridge, &tx_evt),
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }

        if bridge.is_connected() {
            match bridge.poll_reply(2) {
                Ok(Some(reply)) => emit_lines(&tx_evt, &reply),
                Ok(None) => {}
                Err(err) => {
                    let _ = tx_evt.send(SerialEvent::Log(format!("Serial error: {err}")));
                    bridge.disconnect();
                    let _ = tx_evt.send(SerialEvent::Connected(false));
                }
            }
        }
    }
}

fn handle_command(cmd: SerialCommand, bridge: &mut SerialBridge, tx_evt: &Sender<SerialEvent>) {
    match cmd {
        SerialCommand::Connect { address, baud } => {
            if bridge.is_connected() {
                bridge.disconnect();
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
            bridge.disconnect();
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
                send_loaded_gcode_worker(bridge, tx_evt, &lines);
            }
            GcodeSerialCountMode::RxLogs => {
                send_loaded_gcode_worker_rx(bridge, tx_evt, &lines);
            }
        }
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
        SerialCommand::JogStop => match bridge.send_line("M0") {
            Ok(()) => {
                let _ = tx_evt.send(SerialEvent::Log("TX> M0".to_string()));
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
    }
}

fn send_loaded_gcode_worker(
    bridge: &mut SerialBridge,
    tx_evt: &Sender<SerialEvent>,
    lines: &[String],
) {
    if lines.is_empty() {
        let _ = tx_evt.send(SerialEvent::GcodeProgress { sent: 0, total: 0 });
        let _ = tx_evt.send(SerialEvent::Log("No G-code loaded.".to_string()));
        return;
    }

    let total = lines.len();
    let _ = tx_evt.send(SerialEvent::GcodeProgress { sent: 0, total });

    let mut sent = 0_u64;
    let mut acked = 0_u64;
    let mut next_index = 0_usize;
    let mut rx_used = 0_usize;
    let mut in_flight: VecDeque<(usize, usize, String)> = VecDeque::new(); // (bytes, line_no, text)
    let mut last_progress = Instant::now();
    let mut last_status_emit = Instant::now() - Duration::from_millis(STATUS_GUI_UPDATE_MS);

    loop {
        // Character-counting: keep RX near full without overflow.
        while next_index < lines.len() {
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
    let _ = tx_evt.send(SerialEvent::Log(
        "[Printing G-Code] completed successfully".to_string(),
    ));
}

fn send_loaded_gcode_worker_rx(
    bridge: &mut SerialBridge,
    tx_evt: &Sender<SerialEvent>,
    lines: &[String],
) {
    if lines.is_empty() {
        let _ = tx_evt.send(SerialEvent::GcodeProgress { sent: 0, total: 0 });
        let _ = tx_evt.send(SerialEvent::Log("No G-code loaded.".to_string()));
        return;
    }

    let total = lines.len();
    let _ = tx_evt.send(SerialEvent::GcodeProgress { sent: 0, total });

    let mut sent = 0_u64;
    let mut acked = 0_u64;
    let mut next_index = 0_usize;
    let mut rx_used = 0_usize;
    let mut in_flight: VecDeque<(usize, usize, String)> = VecDeque::new(); // (bytes, line_no, text)
    let mut last_progress = Instant::now();
    let mut rx_free_hint: Option<usize> = None;
    let mut reported_fallback = false;
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
        while status_tick_rx.try_recv().is_ok() {
            if let Err(err) = bridge.send_status_query() {
                let _ = status_stop_tx.send(());
                let _ = status_thread.join();
                let _ = tx_evt.send(SerialEvent::Log(format!("Status query failed: {err}")));
                return;
            }
        }

        while next_index < lines.len() {
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

        if next_index >= lines.len() && in_flight.is_empty() {
            break;
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
                            if let Some(rx_free) = report.rx_free {
                                let rx_free = rx_free.min(GRBL_RX_LIMIT);
                                rx_free_hint = Some(rx_free);
                                rx_used = GRBL_RX_LIMIT.saturating_sub(rx_free);
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
                let _ = status_stop_tx.send(());
                let _ = status_thread.join();
                let _ = tx_evt.send(SerialEvent::Log(format!("Send failed: {err}")));
                return;
            }
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
    let _ = tx_evt.send(SerialEvent::Log(
        "[Printing G-Code] completed successfully".to_string(),
    ));
}

#[derive(Debug, Default, Clone, Copy)]
struct GrblStatusReport {
    planner_free: Option<usize>,
    rx_free: Option<usize>,
    position: Option<[f32; 2]>,
}

fn parse_grbl_status_report(line: &str) -> Option<GrblStatusReport> {
    let body = line.strip_prefix('<')?.strip_suffix('>')?;
    let mut report = GrblStatusReport::default();

    for field in body.split('|') {
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
