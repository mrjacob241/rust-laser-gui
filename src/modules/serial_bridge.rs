use super::serial_lib::SerialClient;
use std::time::Duration;

pub struct SerialBridge {
    client: Option<SerialClient>,
}

impl SerialBridge {
    pub fn new() -> Self {
        Self { client: None }
    }

    pub fn is_connected(&self) -> bool {
        self.client.is_some()
    }

    pub fn connect(&mut self, address: &str, baud: u32) -> Result<(), String> {
        let client = SerialClient::open(address, baud)?;
        self.client = Some(client);
        Ok(())
    }

    pub fn disconnect(&mut self) {
        self.client = None;
    }

    pub fn send_line(&mut self, line: &str) -> Result<(), String> {
        let client = self
            .client
            .as_mut()
            .ok_or("Serial not connected".to_string())?;
        client.send_line(line)
    }

    pub fn poll_reply(&mut self, wait_ms: u64) -> Result<Option<String>, String> {
        let client = match self.client.as_mut() {
            Some(client) => client,
            None => return Ok(None),
        };

        let reply = client
            .read_for(Duration::from_millis(wait_ms))
            .map_err(|e| e.to_string())?;
        if reply.trim().is_empty() {
            Ok(None)
        } else {
            Ok(Some(reply.trim_end().to_string()))
        }
    }
}
