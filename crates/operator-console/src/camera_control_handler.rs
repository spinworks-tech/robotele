//! Encode `CameraControl` FlatBuffers <-> `roboprotocol_core::camera_control`
//! types. Operator console only ever encodes -- the control flows one way,
//! operator to robot; robot-edge does the decoding.

use flatbuffers::FlatBufferBuilder;
use roboprotocol_core::camera_control::CameraControl;
use roboprotocol_proto::{CameraControl as FbCameraControl, CameraControlArgs};

pub fn encode_camera_control(control: &CameraControl) -> Vec<u8> {
    let mut b = FlatBufferBuilder::new();
    let offset = FbCameraControl::create(
        &mut b,
        &CameraControlArgs {
            brightness: control.brightness,
            contrast: control.contrast,
            ev: control.ev,
            shutter_us: control.shutter_us,
            control_seq: control.control_seq,
        },
    );
    b.finish(offset, None);
    b.finished_data().to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn camera_control_encodes_to_a_parseable_flatbuffer() {
        let control = CameraControl { brightness: -0.3, contrast: 1.5, ev: 1.0, shutter_us: 12000, control_seq: 6 };
        let bytes = encode_camera_control(&control);
        let decoded = flatbuffers::get_root::<FbCameraControl>(&bytes);
        assert_eq!(decoded.brightness(), -0.3);
        assert_eq!(decoded.contrast(), 1.5);
        assert_eq!(decoded.ev(), 1.0);
        assert_eq!(decoded.shutter_us(), 12000);
        assert_eq!(decoded.control_seq(), 6);
    }
}
