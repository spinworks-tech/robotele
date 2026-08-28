//! Pure protocol logic for RoboProtocol: safety state machine, watchdog,
//! control-source arbitration, HELLO negotiation, robot profile/layout
//! derivation, payload sizing, and timestamp/RTT math.
//!
//! Deliberately has no networking, tokio, or `quiche` dependency (see
//! DESIGN.md §5's stack choice) so it stays trivially unit-testable and
//! reusable by anything that needs the same rules -- `robot-edge`,
//! `operator-console`, and eventually a ROS 2 bridge -- without dragging
//! the transport in.

pub mod action_trigger;
pub mod camera_control;
pub mod datagram;
pub mod estop;
pub mod hello;
pub mod interpolation;
pub mod profile;
pub mod recording;
pub mod safety;
pub mod sizing;
pub mod timestamp;
pub mod video;
