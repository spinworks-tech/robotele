//! Decode `CameraControl` FlatBuffers <-> `roboprotocol_core::camera_control`
//! types. Robot-edge only ever decodes -- the control flows one way,
//! operator to robot; operator-console does the encoding.

use roboprotocol_core::camera_control::CameraControl;
use roboprotocol_proto::CameraControl as FbCameraControl;

pub fn decode_camera_control(buf: &[u8]) -> anyhow::Result<CameraControl> {
    // Same caveat as hello_handler::decode_hello: this crate's pinned old
    // flatbuffers version has no buffer verification, so a malformed
    // buffer is undefined behavior rather than a clean error -- bounded to
    // "our own peer sent us garbage" by the authenticated QUIC/TLS 1.3
    // connection, not an open attack surface.
    let c = flatbuffers::get_root::<FbCameraControl>(buf);
    Ok(CameraControl {
        brightness: c.brightness(),
        contrast: c.contrast(),
        ev: c.ev(),
        shutter_us: c.shutter_us(),
        control_seq: c.control_seq(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use flatbuffers::FlatBufferBuilder;
    use roboprotocol_proto::CameraControlArgs;

    #[test]
    fn decodes_a_well_formed_camera_control() {
        let mut b = FlatBufferBuilder::new();
        let offset = FbCameraControl::create(
            &mut b,
            &CameraControlArgs { brightness: 0.2, contrast: 1.3, ev: -0.5, shutter_us: 8000, control_seq: 4 },
        );
        b.finish(offset, None);
        let control = decode_camera_control(b.finished_data()).unwrap();
        assert_eq!(control.brightness, 0.2);
        assert_eq!(control.contrast, 1.3);
        assert_eq!(control.ev, -0.5);
        assert_eq!(control.shutter_us, 8000);
        assert_eq!(control.control_seq, 4);
    }
}
