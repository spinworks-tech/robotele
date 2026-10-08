//! Synthetic sensors for `--sim-sensor` (docs/13-large-sensor-payloads.md).
//!
//! The XGO-Lite V2 reference robot has no lidar, radar or depth camera, so
//! this ray-casts a small scene instead: a 20 m x 12 m room with a floor and
//! ceiling, two static pillars, and a "person" walking back and forth (which
//! gives radar something with Doppler). Each sensor runs on its own thread at
//! its descriptor's `max_hz` and hands frames to the session through
//! `SensorRx`, which keeps only the latest frame per sensor -- the same
//! "latest wins" rule as the video sender, so a slow session never builds a
//! backlog of stale frames.

use std::collections::BTreeMap;
use std::f32::consts::TAU;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use tokio::sync::Notify;

use roboprotocol_core::sensor::{
    DepthFormat, DepthIntrinsics, Point, PointFormat, RangeImageFormat, SensorDescriptor, SensorEncoding,
};
use roboprotocol_core::timestamp::now_micros;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimSensorKind {
    /// Navigation point cloud: 16 beams x 720 columns, ~10k points, 10 Hz.
    Cloud,
    /// 3D lidar range image: 64 beams x 1,024 columns, 10 Hz.
    Lidar,
    /// Radar detections with Doppler and SNR: 3 x 100 rays ahead, 20 Hz.
    Radar,
    /// Depth camera: 320 x 240, 15 Hz.
    Depth,
}

/// All sensors sit at this height above the floor, facing +x.
const SENSOR_HEIGHT_M: f32 = 0.5;
const CEILING_M: f32 = 2.5;
const ROOM_X_M: (f32, f32) = (-8.0, 12.0);
const ROOM_Y_M: (f32, f32) = (-6.0, 6.0);
const PILLAR_HEIGHT_M: f32 = 1.8;
const MAX_RANGE_M: f32 = 100.0;
const DEPTH_MAX_RANGE_M: f32 = 10.0;

const LIDAR_BEAMS: u16 = 64;
const LIDAR_COLUMNS: u16 = 1024;
const CLOUD_BEAMS: usize = 16;
const CLOUD_COLUMNS: usize = 720;
const RADAR_ELEVATIONS_DEG: [f32; 3] = [-2.0, 0.0, 2.0];
const RADAR_AZIMUTHS: usize = 100;
const RADAR_FOV_DEG: f32 = 120.0;
const DEPTH_W: u16 = 320;
const DEPTH_H: u16 = 240;
const DEPTH_HFOV_DEG: f32 = 60.0;

impl SimSensorKind {
    pub fn parse(name: &str) -> Option<Self> {
        match name {
            "cloud" => Some(Self::Cloud),
            "lidar" => Some(Self::Lidar),
            "radar" => Some(Self::Radar),
            "depth" => Some(Self::Depth),
            _ => None,
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Cloud => "sim cloud",
            Self::Lidar => "sim lidar",
            Self::Radar => "sim radar",
            Self::Depth => "sim depth",
        }
    }

    pub fn descriptor(self, sensor_id: u8) -> SensorDescriptor {
        let (encoding, max_hz, max_bitrate_kbps) = match self {
            Self::Cloud => (SensorEncoding::Points(PointFormat { scale_m: 0.01, intensity: true, doppler_scale_mps: None, snr: false }), 10, 8_000),
            Self::Lidar => (
                SensorEncoding::RangeImage {
                    format: RangeImageFormat { beams: LIDAR_BEAMS, columns: LIDAR_COLUMNS, range_scale_m: 0.002, intensity: true },
                    beam_elevations_rad: lidar_elevations(),
                },
                10,
                16_000,
            ),
            Self::Radar => (SensorEncoding::Points(PointFormat { scale_m: 0.01, intensity: false, doppler_scale_mps: Some(0.01), snr: true }), 20, 500),
            Self::Depth => (SensorEncoding::Depth { format: DepthFormat::new(DEPTH_W, DEPTH_H), intrinsics: depth_intrinsics() }, 15, 19_000),
        };
        SensorDescriptor {
            sensor_id,
            label: self.label().to_string(),
            encoding,
            max_hz,
            max_bitrate_kbps,
            mount_position_m: [0.0, 0.0, SENSOR_HEIGHT_M],
            mount_orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

/// ±22.5°, evenly spaced (a 45° vertical field of view).
fn lidar_elevations() -> Vec<f32> {
    let n = LIDAR_BEAMS as usize;
    (0..n).map(|b| (-22.5 + 45.0 * b as f32 / (n - 1) as f32).to_radians()).collect()
}

fn depth_intrinsics() -> DepthIntrinsics {
    let f = (DEPTH_W as f32 / 2.0) / (DEPTH_HFOV_DEG.to_radians() / 2.0).tan();
    DepthIntrinsics { fx: f, fy: f, cx: DEPTH_W as f32 / 2.0, cy: DEPTH_H as f32 / 2.0 }
}

struct Pillar {
    x: f32,
    y: f32,
    radius: f32,
    /// Horizontal velocity, m/s -- non-zero only for the walking person.
    vx: f32,
    vy: f32,
}

struct Hit {
    range_m: f32,
    intensity: u8,
    /// Target velocity along the ray, m/s (positive = moving away).
    radial_mps: f32,
}

/// The scene `t_s` seconds after the sensors started.
struct Scene {
    pillars: [Pillar; 3],
}

impl Scene {
    fn at(t_s: f32) -> Self {
        // Walks x = 4 ± 2 m at up to 1 m/s.
        let w = 0.5;
        Self {
            pillars: [
                Pillar { x: 3.0, y: 2.0, radius: 0.4, vx: 0.0, vy: 0.0 },
                Pillar { x: 6.0, y: -2.5, radius: 0.6, vx: 0.0, vy: 0.0 },
                Pillar { x: 4.0 + 2.0 * (w * t_s).sin(), y: -0.5, radius: 0.3, vx: 2.0 * w * (w * t_s).cos(), vy: 0.0 },
            ],
        }
    }

    /// Casts a ray from the sensor along unit vector `d` (sensor frame,
    /// z up), returning the nearest hit within `MAX_RANGE_M`.
    fn cast(&self, d: [f32; 3]) -> Option<Hit> {
        let [dx, dy, dz] = d;
        let mut best = Hit { range_m: f32::INFINITY, intensity: 0, radial_mps: 0.0 };
        let mut consider = |t: f32, intensity: u8, radial_mps: f32| {
            if t > 0.0 && t < best.range_m {
                best = Hit { range_m: t, intensity, radial_mps };
            }
        };
        if dz < 0.0 {
            consider(-SENSOR_HEIGHT_M / dz, 40, 0.0);
        } else if dz > 0.0 {
            consider((CEILING_M - SENSOR_HEIGHT_M) / dz, 60, 0.0);
        }
        // The sensor is inside the room, so the nearest wall plane along
        // the ray is where the ray leaves the room.
        for (d_axis, (lo, hi)) in [(dx, ROOM_X_M), (dy, ROOM_Y_M)] {
            if d_axis > 0.0 {
                consider(hi / d_axis, 120, 0.0);
            } else if d_axis < 0.0 {
                consider(lo / d_axis, 120, 0.0);
            }
        }
        for p in &self.pillars {
            // |t (dx, dy) - (x, y)|^2 = r^2, nearer root.
            let a = dx * dx + dy * dy;
            let b = -2.0 * (dx * p.x + dy * p.y);
            let c = p.x * p.x + p.y * p.y - p.radius * p.radius;
            let disc = b * b - 4.0 * a * c;
            if a > 1e-9 && disc >= 0.0 {
                let t = (-b - disc.sqrt()) / (2.0 * a);
                let z = t * dz;
                if (-SENSOR_HEIGHT_M..=PILLAR_HEIGHT_M - SENSOR_HEIGHT_M).contains(&z) {
                    consider(t, 220, p.vx * dx + p.vy * dy);
                }
            }
        }
        (best.range_m <= MAX_RANGE_M).then_some(best)
    }
}

fn direction(azimuth: f32, elevation: f32) -> [f32; 3] {
    let (sin_az, cos_az) = azimuth.sin_cos();
    let (sin_el, cos_el) = elevation.sin_cos();
    [cos_el * cos_az, cos_el * sin_az, sin_el]
}

/// Renders one frame's elements for `descriptor`.
fn render(kind: SimSensorKind, descriptor: &SensorDescriptor, scene: &Scene) -> Vec<u8> {
    match (&descriptor.encoding, kind) {
        (SensorEncoding::RangeImage { format, beam_elevations_rad }, _) => {
            let (beams, columns) = (format.beams as usize, format.columns as usize);
            let mut ranges = vec![0.0; beams * columns];
            let mut intensity = vec![0u8; beams * columns];
            for (b, &el) in beam_elevations_rad.iter().enumerate() {
                for c in 0..columns {
                    if let Some(hit) = scene.cast(direction(TAU * c as f32 / columns as f32, el)) {
                        ranges[b * columns + c] = hit.range_m;
                        intensity[b * columns + c] = hit.intensity;
                    }
                }
            }
            format.encode(&ranges, Some(&intensity)).expect("dimensions come from the same format")
        }
        (SensorEncoding::Points(format), SimSensorKind::Radar) => {
            let mut points = Vec::with_capacity(RADAR_ELEVATIONS_DEG.len() * RADAR_AZIMUTHS);
            for el in RADAR_ELEVATIONS_DEG.map(f32::to_radians) {
                for i in 0..RADAR_AZIMUTHS {
                    let az = (-RADAR_FOV_DEG / 2.0 + RADAR_FOV_DEG * i as f32 / (RADAR_AZIMUTHS - 1) as f32).to_radians();
                    let d = direction(az, el);
                    if let Some(hit) = scene.cast(d) {
                        points.push(Point {
                            x: hit.range_m * d[0],
                            y: hit.range_m * d[1],
                            z: hit.range_m * d[2],
                            doppler_mps: hit.radial_mps,
                            snr: (40.0 - 1.5 * hit.range_m).clamp(1.0, 40.0) as u8,
                            ..Point::default()
                        });
                    }
                }
            }
            format.encode(&points).0
        }
        (SensorEncoding::Points(format), _) => {
            let mut points = Vec::with_capacity(CLOUD_BEAMS * CLOUD_COLUMNS);
            for b in 0..CLOUD_BEAMS {
                let el = (-15.0 + 30.0 * b as f32 / (CLOUD_BEAMS - 1) as f32).to_radians();
                for c in 0..CLOUD_COLUMNS {
                    let d = direction(TAU * c as f32 / CLOUD_COLUMNS as f32, el);
                    if let Some(hit) = scene.cast(d) {
                        points.push(Point { x: hit.range_m * d[0], y: hit.range_m * d[1], z: hit.range_m * d[2], intensity: hit.intensity, ..Point::default() });
                    }
                }
            }
            format.encode(&points).0
        }
        (SensorEncoding::Depth { format, intrinsics }, _) => {
            let (w, h) = (format.width as usize, format.height as usize);
            let mut depth_mm = vec![0u16; w * h];
            for v in 0..h {
                for u in 0..w {
                    // Camera looks along +x; image u grows to -y, v to -z.
                    let ray = [1.0, -(u as f32 + 0.5 - intrinsics.cx) / intrinsics.fx, -(v as f32 + 0.5 - intrinsics.cy) / intrinsics.fy];
                    let norm = (ray[0] * ray[0] + ray[1] * ray[1] + ray[2] * ray[2]).sqrt();
                    let d = ray.map(|x| x / norm);
                    if let Some(hit) = scene.cast(d) {
                        let z = hit.range_m * d[0];
                        if z <= DEPTH_MAX_RANGE_M {
                            depth_mm[v * w + u] = (z * 1000.0).round() as u16;
                        }
                    }
                }
            }
            format.encode(&depth_mm).expect("dimensions come from the same format")
        }
    }
}

/// `--sim-map`: the sim room as an occupancy grid seen from above, `t_s`
/// seconds in (the walking person moves between versions). Cell size is
/// `resolution_m`; the grid covers the room with the walls as its border.
pub fn occupancy_grid(t_s: f32, resolution_m: f32) -> roboprotocol_core::bulk::OccupancyGrid {
    let (x0, x1) = ROOM_X_M;
    let (y0, y1) = ROOM_Y_M;
    let width = ((x1 - x0) / resolution_m).round() as u32;
    let height = ((y1 - y0) / resolution_m).round() as u32;
    let scene = Scene::at(t_s);
    let mut cells = vec![0u8; (width * height) as usize];
    for row in 0..height {
        for col in 0..width {
            let border = row == 0 || col == 0 || row == height - 1 || col == width - 1;
            let x = x0 + (col as f32 + 0.5) * resolution_m;
            let y = y0 + (row as f32 + 0.5) * resolution_m;
            let pillar = scene.pillars.iter().any(|p| (x - p.x).powi(2) + (y - p.y).powi(2) <= p.radius * p.radius);
            if border || pillar {
                cells[(row * width + col) as usize] = 100;
            }
        }
    }
    roboprotocol_core::bulk::OccupancyGrid {
        width,
        height,
        resolution_mm: (resolution_m * 1000.0).round() as u32,
        origin_x_mm: (x0 * 1000.0) as i32,
        origin_y_mm: (y0 * 1000.0) as i32,
        cells,
    }
}

/// One frame's elements, ready for `sensor::slice_frame`.
pub struct SensorFrame {
    pub sensor_id: u8,
    pub frame_seq: u32,
    pub capture_time_us: u64,
    pub elements: Vec<u8>,
}

/// The session's end of the sensor threads. Holds at most one frame per
/// sensor: a frame not yet taken by `recv` is replaced by the next one.
/// Dropping it stops the threads.
pub struct SensorRx {
    slots: Arc<Mutex<BTreeMap<u8, SensorFrame>>>,
    notify: Arc<Notify>,
    stop: Arc<AtomicBool>,
}

impl SensorRx {
    /// Waits until at least one sensor has a new frame, then takes the
    /// latest frame of every sensor that has one.
    pub async fn recv(&mut self) -> Vec<SensorFrame> {
        loop {
            {
                let mut slots = self.slots.lock().expect("sensor slot lock poisoned");
                if !slots.is_empty() {
                    return std::mem::take(&mut *slots).into_values().collect();
                }
            }
            self.notify.notified().await;
        }
    }
}

impl Drop for SensorRx {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

/// Starts one thread per sensor. `sensors` pairs each kind with the
/// descriptor advertised for it.
pub fn spawn(sensors: &[(SimSensorKind, SensorDescriptor)]) -> SensorRx {
    let slots = Arc::new(Mutex::new(BTreeMap::new()));
    let notify = Arc::new(Notify::new());
    let stop = Arc::new(AtomicBool::new(false));
    let start = Instant::now();
    for (kind, descriptor) in sensors.iter().cloned() {
        let (slots, notify, stop) = (slots.clone(), notify.clone(), stop.clone());
        std::thread::Builder::new()
            .name(format!("sim-sensor-{}", descriptor.sensor_id))
            .spawn(move || {
                let period = Duration::from_secs_f32(1.0 / descriptor.max_hz as f32);
                let mut next = Instant::now();
                let mut frame_seq = 0u32;
                while !stop.load(Ordering::Relaxed) {
                    let capture_time_us = now_micros();
                    let elements = render(kind, &descriptor, &Scene::at(start.elapsed().as_secs_f32()));
                    let frame = SensorFrame { sensor_id: descriptor.sensor_id, frame_seq, capture_time_us, elements };
                    slots.lock().expect("sensor slot lock poisoned").insert(descriptor.sensor_id, frame);
                    notify.notify_one();
                    frame_seq = frame_seq.wrapping_add(1);
                    next += period;
                    // If rendering overran, start the next frame now rather
                    // than bursting to catch up.
                    next = next.max(Instant::now());
                    std::thread::sleep(next.saturating_duration_since(Instant::now()));
                }
            })
            .expect("spawning a sim sensor thread");
    }
    SensorRx { slots, notify, stop }
}

#[cfg(test)]
mod tests {
    use super::*;
    use roboprotocol_core::sensor::{slice_count, AssembledFrame, SensorData, DEFAULT_SLICE_PAYLOAD};

    fn decode(kind: SimSensorKind) -> SensorData {
        let descriptor = kind.descriptor(0);
        assert_eq!(descriptor.validate(), Ok(()));
        let elements = render(kind, &descriptor, &Scene::at(0.0));
        let frame = AssembledFrame { sensor_id: 0, frame_seq: 0, capture_time_us: 0, slices: vec![Some(elements)] };
        descriptor.encoding.decode(&frame)
    }

    #[test]
    fn every_kind_parses_and_has_a_valid_descriptor() {
        for name in ["cloud", "lidar", "radar", "depth"] {
            let kind = SimSensorKind::parse(name).unwrap();
            assert_eq!(kind.descriptor(3).validate(), Ok(()), "{name}");
        }
        assert!(SimSensorKind::parse("sonar").is_none());
    }

    #[test]
    fn lidar_sees_the_floor_below_and_walls_all_around() {
        let SensorData::RangeImage(img) = decode(SimSensorKind::Lidar) else { panic!("range image expected") };
        let columns = img.columns as usize;
        // Lowest beam, straight ahead: the floor, at a slant range of
        // 0.5 / sin(22.5°) ≈ 1.31 m.
        assert!((img.ranges_m[0] - 1.307).abs() < 0.01, "{}", img.ranges_m[0]);
        // Horizontal-ish middle beam, facing -x: the wall at x = -8.
        let mid = LIDAR_BEAMS as usize / 2;
        let back = img.ranges_m[mid * columns + columns / 2];
        assert!((back - 8.0).abs() < 0.2, "{back}");
        assert!(img.ranges_m.iter().all(|&r| r > 0.0), "a closed room returns every ray");
    }

    #[test]
    fn cloud_is_about_ten_thousand_points_as_doc_13_sizes_it() {
        let SensorData::Points(points) = decode(SimSensorKind::Cloud) else { panic!("points expected") };
        assert!((10_000..=CLOUD_BEAMS * CLOUD_COLUMNS).contains(&points.len()), "{}", points.len());
        let element_size = SimSensorKind::Cloud.descriptor(0).encoding.element_size();
        assert!(slice_count(points.len(), element_size, DEFAULT_SLICE_PAYLOAD).unwrap() <= 75);
    }

    #[test]
    fn radar_reports_doppler_only_for_the_moving_person() {
        // At t = 0 the person is at x = 4 moving +x at 1 m/s.
        let SensorData::Points(points) = decode(SimSensorKind::Radar) else { panic!("points expected") };
        assert_eq!(points.len(), RADAR_ELEVATIONS_DEG.len() * RADAR_AZIMUTHS);
        let moving: Vec<_> = points.iter().filter(|p| p.doppler_mps.abs() > 0.05).collect();
        assert!(!moving.is_empty());
        assert!(moving.iter().all(|p| (3.0..4.5).contains(&p.x) && p.doppler_mps > 0.9));
    }

    #[test]
    fn depth_centre_pixel_sees_the_person_and_far_walls_are_cut_off() {
        let SensorData::Depth(img) = decode(SimSensorKind::Depth) else { panic!("depth expected") };
        let (w, h) = (DEPTH_W as usize, DEPTH_H as usize);
        // Straight ahead, slightly right (-y) toward the person at y = -0.5:
        // their near surface is about 3.7 m away.
        let u = (w as f32 / 2.0 + 0.5 * depth_intrinsics().fx / 3.7) as usize;
        let d = img.depth_mm[(h / 2) * w + u];
        assert!((3_600..3_800).contains(&d), "{d}");
        // Dead centre misses every pillar and meets the wall at 12 m,
        // beyond the 10 m cutoff.
        assert_eq!(img.depth_mm[(h / 2) * w + w / 2], 0);
    }

    #[test]
    fn occupancy_grid_marks_walls_and_pillars_and_follows_the_person() {
        let g = occupancy_grid(0.0, 0.02);
        assert_eq!((g.width, g.height, g.resolution_mm), (1000, 600, 20));
        let cell = |g: &roboprotocol_core::bulk::OccupancyGrid, x: f32, y: f32| {
            let col = ((x - ROOM_X_M.0) / 0.02) as u32;
            let row = ((y - ROOM_Y_M.0) / 0.02) as u32;
            g.cells[(row * g.width + col) as usize]
        };
        assert_eq!(cell(&g, 3.0, 2.0), 100, "static pillar");
        assert_eq!(cell(&g, 0.0, 0.0), 0, "the robot's own spot is free");
        assert_eq!(g.cells[0], 100, "wall");
        assert_eq!(cell(&g, 4.0, -0.5), 100, "person at x = 4 at t = 0");
        let later = occupancy_grid(std::f32::consts::PI, 0.02); // the person has walked to x ~ 6
        assert_eq!(cell(&later, 4.0, -0.5), 0);
    }

    #[tokio::test]
    async fn sensor_rx_hands_over_frames_and_stops_on_drop() {
        let sensors = [(SimSensorKind::Radar, SimSensorKind::Radar.descriptor(5))];
        let mut rx = spawn(&sensors);
        let frames = tokio::time::timeout(Duration::from_secs(2), rx.recv()).await.expect("a frame within 2 s");
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].sensor_id, 5);
        let stop = rx.stop.clone();
        drop(rx);
        assert!(stop.load(Ordering::Relaxed));
    }
}
