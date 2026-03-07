use super::serial_bridge::SerialBridge;
use std::collections::VecDeque;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;
use std::time::{Duration, Instant};

const GRBL_RX_LIMIT: usize = 127;
const STATUS_GUI_UPDATE_MS: u64 = 500;
const STREAM_POLL_MS: u64 = 20;
const STREAM_STALL_TIMEOUT_SECS: u64 = 30;

pub enum SerialCommand {
    Connect {
        address: String,
        baud: u32,
    },
    Disconnect,
    SendLine(String),
    SendLoadedGcode(Vec<String>),
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
        SerialCommand::SendLoadedGcode(lines) => {
            send_loaded_gcode_worker(bridge, tx_evt, &lines);
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
        let _ = tx_evt.send(SerialEvent::Log("No G-code loaded.".to_string()));
        return;
    }

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
}

fn emit_lines(tx_evt: &Sender<SerialEvent>, raw: &str) {
    for line in raw.lines() {
        let text = line.trim();
        if !text.is_empty() {
            let _ = tx_evt.send(SerialEvent::Log(format!("RX> {text}")));
        }
    }
}
