//! JSON-lines message types for the `xgo_bridge.py` IPC protocol.
//! Must stay in lockstep with `xgo_bridge/xgo_bridge.py`'s docstring.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
pub enum BridgeCommand {
    Move { x: f64, y: f64, seq: u64 },
    Turn { step: f64, seq: u64 },
    Attitude { axis: char, data: f64, seq: u64 },
    Action { id: u8, seq: u64 },
    Motor { id: u8, angle: f64, seq: u64 },
    /// Cartesian end-effector target, mm relative to base: x [-80,155], z [-95,155].
    Arm { x: f64, z: f64, seq: u64 },
    /// Gripper position: 0 = open, 255 = closed.
    Claw { pos: u8, seq: u64 },
    Stop { seq: u64 },
    Heartbeat { seq: u64 },
    Query { seq: u64 },
    Estop { seq: u64 },
    EstopClear { seq: u64 },
}

impl BridgeCommand {
    pub fn seq(&self) -> u64 {
        match self {
            BridgeCommand::Move { seq, .. }
            | BridgeCommand::Turn { seq, .. }
            | BridgeCommand::Attitude { seq, .. }
            | BridgeCommand::Action { seq, .. }
            | BridgeCommand::Motor { seq, .. }
            | BridgeCommand::Arm { seq, .. }
            | BridgeCommand::Claw { seq, .. }
            | BridgeCommand::Stop { seq }
            | BridgeCommand::Heartbeat { seq }
            | BridgeCommand::Query { seq }
            | BridgeCommand::Estop { seq }
            | BridgeCommand::EstopClear { seq } => *seq,
        }
    }

    pub fn to_line(&self) -> anyhow::Result<String> {
        Ok(serde_json::to_string(self)?)
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum BridgeEvent {
    Ack {
        seq: Option<u64>,
        ok: bool,
        #[serde(default)]
        error: Option<String>,
    },
    Telemetry {
        ts: f64,
        motors: Vec<f64>,
        battery: u8,
        roll: f64,
        pitch: f64,
        yaw: f64,
    },
    Status {
        state: String,
        #[serde(default)]
        detail: Option<String>,
    },
    Log {
        level: String,
        msg: String,
    },
}

impl BridgeEvent {
    pub fn from_line(line: &str) -> anyhow::Result<Self> {
        Ok(serde_json::from_str(line)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn move_command_serializes_to_the_documented_wire_shape() {
        let cmd = BridgeCommand::Move { x: 10.0, y: -5.0, seq: 42 };
        let line = cmd.to_line().unwrap();
        let value: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(value["cmd"], "move");
        assert_eq!(value["x"], 10.0);
        assert_eq!(value["y"], -5.0);
        assert_eq!(value["seq"], 42);
    }

    #[test]
    fn estop_clear_renders_as_snake_case() {
        let cmd = BridgeCommand::EstopClear { seq: 1 };
        assert!(cmd.to_line().unwrap().contains("\"cmd\":\"estop_clear\""));
    }

    #[test]
    fn parses_ack_telemetry_status_and_log_events() {
        let ack: BridgeEvent = BridgeEvent::from_line(r#"{"type":"ack","seq":5,"ok":true}"#).unwrap();
        assert!(matches!(ack, BridgeEvent::Ack { seq: Some(5), ok: true, error: None }));

        let telemetry: BridgeEvent = BridgeEvent::from_line(
            r#"{"type":"telemetry","ts":1.0,"motors":[0.0,1.0],"battery":80,"roll":0.1,"pitch":0.2,"yaw":0.3}"#,
        )
        .unwrap();
        assert!(matches!(telemetry, BridgeEvent::Telemetry { battery: 80, .. }));

        let status: BridgeEvent = BridgeEvent::from_line(r#"{"type":"status","state":"estopped","detail":"watchdog"}"#).unwrap();
        assert!(matches!(status, BridgeEvent::Status { ref state, .. } if state == "estopped"));

        let log: BridgeEvent = BridgeEvent::from_line(r#"{"type":"log","level":"warn","msg":"hi"}"#).unwrap();
        assert!(matches!(log, BridgeEvent::Log { .. }));
    }
}
