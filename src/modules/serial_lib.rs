use serialport::SerialPort;
use std::io::{self, Write};
use std::time::{Duration, Instant};

pub const DEFAULT_PORT: &str = "/dev/ttyUSB0";
pub const DEFAULT_BAUD: u32 = 115_200;
pub const READ_TIMEOUT_MS: u64 = 200;

pub struct SerialClient {
    port: Box<dyn SerialPort>,
}

impl SerialClient {
    pub fn open(path: &str, baud: u32) -> Result<Self, String> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(READ_TIMEOUT_MS))
            .open()
            .map_err(|e| format!("Failed to open {path} at {baud} baud: {e}"))?;
        Ok(Self { port })
    }

    pub fn send_raw(&mut self, payload: &[u8]) -> Result<(), String> {
        self.port
            .write_all(payload)
            .map_err(|e| format!("Write failed: {e}"))?;
        self.port.flush().map_err(|e| format!("Flush failed: {e}"))
    }

    pub fn send_line(&mut self, line: &str) -> Result<(), String> {
        let mut payload = line.as_bytes().to_vec();
        payload.push(b'\n');
        self.send_raw(&payload)
    }

    pub fn send_status_query(&mut self) -> Result<(), String> {
        self.send_raw(b"?")
    }

    pub fn send_feed_hold(&mut self) -> Result<(), String> {
        self.send_raw(b"!")
    }

    pub fn send_spindle_stop(&mut self) -> Result<(), String> {
        self.send_raw(&[0x9E])
    }

    pub fn send_soft_reset(&mut self) -> Result<(), String> {
        self.send_raw(&[0x18])
    }

    pub fn send_cycle_start(&mut self) -> Result<(), String> {
        self.send_raw(b"~")
    }

    pub fn read_for(&mut self, duration: Duration) -> io::Result<String> {
        let deadline = Instant::now() + duration;
        let mut buffer = [0_u8; 256];
        let mut out = Vec::new();

        while Instant::now() < deadline {
            match self.port.read(&mut buffer) {
                Ok(n) if n > 0 => out.extend_from_slice(&buffer[..n]),
                Ok(_) => {}
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
                Err(e) => return Err(e),
            }
        }

        Ok(String::from_utf8_lossy(&out).to_string())
    }
}
