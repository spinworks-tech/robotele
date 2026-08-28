//! Robot & media session descriptor types (DESIGN.md §1.3, REQUIREMENTS.md FR-7).
//!
//! Pure data + layout-derivation logic. Wire encode/decode into the
//! `SessionDescribe`/`SessionAccept` FlatBuffers tables lives in
//! `roboprotocol-proto`; this module is what both endpoints compute the
//! *same* answer from (§1.3.3 -- "layout is derived, not transmitted").

use crate::sizing::QuantizationTier;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointDescriptor {
    pub min_limit: f32,
    pub max_limit: f32,
    pub max_velocity: f32,
    pub has_torque_sensing: bool,
    pub region_id: u8,
}

/// FR-1.5 (REQUIREMENTS.md), DESIGN.md §1.3.2/§1.3.3: what a Channel B
/// *command* datagram means for a region. Telemetry is always per-joint
/// regardless of this -- it only governs command encoding.
///
/// Explicit discriminants match the FlatBuffers `CommandShape` enum
/// ordering in `roboprotocol-proto`'s schema; this cast (`as u8`) is what
/// `RobotProfile::profile_hash` and the FlatBuffers encoders rely on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CommandShape {
    /// One command field per joint in the region (the pre-existing model).
    Kinematic = 0,
    /// Body-frame velocity/attitude. Unlike the other two shapes this is
    /// inherently a whole-robot quantity, not a per-region one -- a legged
    /// or wheeled base doesn't have an independent velocity per leg/wheel
    /// -- so every region tagged `VelocityAttitude` shares ONE command
    /// block for the robot rather than each contributing its own
    /// (§1.3.3's collapsing rule, see `derive_command_layout`).
    VelocityAttitude = 1,
    /// Target position + gripper, one independent block per region (e.g.
    /// a dual-arm robot's two arms are independently targetable).
    CartesianEndEffector = 2,
}

impl CommandShape {
    /// Fixed field count for shapes whose command layout doesn't scale
    /// with joint count. `None` for `Kinematic`, whose field count is the
    /// region's `joint_count` instead.
    pub const fn fixed_field_count(self) -> Option<u32> {
        match self {
            CommandShape::Kinematic => None,
            CommandShape::VelocityAttitude => Some(6), // vx,vy,turn,roll,pitch,yaw (action_id moved to Channel C, FR-1.8)
            CommandShape::CartesianEndEffector => Some(4), // x,y,z,gripper
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct BodyRegionDescriptor {
    pub region_id: u8,
    /// Debug/logging only -- not used for wire offsets (DESIGN.md §1.3.2).
    pub name: String,
    pub joint_start: u16,
    pub joint_count: u16,
    pub has_force_torque_sensor: bool,
    pub command_shape: CommandShape,
}

/// DESIGN.md §5 specifies H.265/AV1 for Channel A; `H264` is a documented
/// v0 deviation (the CM4's BCM2711 hardware encoder path via
/// `libcamera`/`rpicam-vid` -- H.265/AV1 hardware encode isn't available on
/// this SoC). Migrating to H.265/AV1 is a later hardware-dependent decision.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Codec {
    H265,
    Av1,
    H264,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CameraDescriptor {
    pub camera_id: u8,
    pub label: String,
    pub codec: Codec,
    pub resolution_w: u16,
    pub resolution_h: u16,
    pub max_fps: u8,
    pub min_bitrate_kbps: u32,
    pub max_bitrate_kbps: u32,
}

/// FR-1.6, DESIGN.md §1.3.6: one whole-robot morphology fact, distinct
/// from `CommandShape` (which is per-region). A hardware fact, not a
/// negotiable capability -- travels in `RobotProfile`/`SESSION_DESCRIBE`,
/// not the `HELLO` capability bitmask.
///
/// Explicit discriminants match the FlatBuffers `BaseType` enum ordering,
/// the same convention `CommandShape` already uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BaseType {
    /// Fixed-base manipulator, no locomotion at all.
    Stationary = 0,
    /// Differential/Ackermann drive -- lateral motion not independently commandable.
    WheeledStandard = 1,
    /// Mecanum/omni wheels -- lateral motion independently commandable.
    WheeledHolonomic = 2,
    /// 2 legs, dynamically balanced -- SR-1's WBC/ZMP override is load-bearing.
    BipedLegs = 3,
    /// 4+ legs, statically stable -- SR-1's WBC/ZMP override is a documented no-op.
    QuadrupedLegs = 4,
    /// A morphology not covered above (e.g. tracked, aerial) -- reserved
    /// for forward compatibility rather than forcing a mismatched value.
    Other = 5,
}

impl BaseType {
    /// SR-1 (REQUIREMENTS.md): the WBC/ZMP Balance Override is load-bearing
    /// only for a dynamically-balanced base. Every other morphology here is
    /// statically stable (or immobile), so it's a documented no-op for
    /// them -- including `Other`, whose applicability isn't classified yet.
    pub const fn requires_dynamic_balance_override(self) -> bool {
        matches!(self, BaseType::BipedLegs)
    }

    /// FR-1.6.2: a `WheeledStandard` base cannot physically strafe, so its
    /// `VelocityAttitudeCommand.vy` is always 0 -- the operator console
    /// should not present lateral-motion control for it.
    pub const fn lateral_velocity_meaningful(self) -> bool {
        !matches!(self, BaseType::WheeledStandard)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct RobotProfile {
    pub dof_count: u16,
    /// Wire order == Channel B command/telemetry field order (§1.3.2).
    pub joints: Vec<JointDescriptor>,
    /// Maps to the §2.2.1 body-region split groups.
    pub regions: Vec<BodyRegionDescriptor>,
    pub base_type: BaseType,
}

impl RobotProfile {
    /// 64-bit hash of the canonical profile, for `SESSION_ACCEPT{cached}`
    /// reuse (§1.3.4). This is an internal cache key, not required to match
    /// the FlatBuffers wire bytes -- only to be stable and collision-free
    /// enough across repeated runs of the same profile.
    pub fn profile_hash(&self) -> u64 {
        let mut bytes = Vec::with_capacity(9 + self.joints.len() * 14 + self.regions.len() * 16);
        bytes.extend_from_slice(&self.dof_count.to_le_bytes());
        bytes.push(self.base_type as u8);
        for j in &self.joints {
            bytes.extend_from_slice(&j.min_limit.to_le_bytes());
            bytes.extend_from_slice(&j.max_limit.to_le_bytes());
            bytes.extend_from_slice(&j.max_velocity.to_le_bytes());
            bytes.push(j.has_torque_sensing as u8);
            bytes.push(j.region_id);
        }
        for r in &self.regions {
            bytes.push(r.region_id);
            bytes.extend_from_slice(r.name.as_bytes());
            bytes.extend_from_slice(&r.joint_start.to_le_bytes());
            bytes.extend_from_slice(&r.joint_count.to_le_bytes());
            bytes.push(r.has_force_torque_sensor as u8);
            bytes.push(r.command_shape as u8);
        }
        fnv1a_64(&bytes)
    }
}

const FNV_OFFSET_BASIS: u64 = 0xcbf29ce484222325;
const FNV_PRIME: u64 = 0x100000001b3;

fn fnv1a_64(data: &[u8]) -> u64 {
    let mut hash = FNV_OFFSET_BASIS;
    for &b in data {
        hash ^= b as u64;
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    hash
}

/// One field's byte-offset/length within a Channel B datagram's packed
/// field vector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FieldOffset {
    pub joint_index: u16,
    pub byte_offset: u32,
    pub byte_len: u32,
}

/// §1.3.3: both endpoints derive the identical byte-offset layout from the
/// negotiated `RobotProfile`, selected body regions, and quantization tier
/// -- never transmitted as an explicit per-field register table. Joints are
/// walked in profile wire order and filtered to `region_ids` (empty slice =
/// all regions), so two independently-implemented endpoints given the same
/// inputs must produce byte-identical output.
pub fn derive_layout(profile: &RobotProfile, region_ids: &[u8], tier: QuantizationTier) -> Vec<FieldOffset> {
    let mut offsets = Vec::new();
    let mut cursor: u32 = 0;
    for (i, joint) in profile.joints.iter().enumerate() {
        if !region_ids.is_empty() && !region_ids.contains(&joint.region_id) {
            continue;
        }
        let byte_len = tier.bytes_per_field();
        offsets.push(FieldOffset { joint_index: i as u16, byte_offset: cursor, byte_len });
        cursor += byte_len;
    }
    offsets
}

/// One block within a derived Channel B **command** layout (FR-1.5,
/// DESIGN.md §1.3.3). Unlike `derive_layout` (always per-joint, used for
/// telemetry regardless of command shape), this is shape-aware per region
/// -- except `VelocityAttitude`, which collapses every selected region
/// tagged with it into one shared block, since body velocity isn't a
/// per-leg/per-wheel quantity.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CommandBlock {
    /// One field per joint in this region, exactly as `derive_layout`
    /// would compute for it.
    Kinematic { region_id: u8, joint_offsets: Vec<FieldOffset> },
    /// The one shared body-velocity/attitude block for the whole robot.
    VelocityAttitude { byte_offset: u32, byte_len: u32 },
    /// One independent Cartesian target block for this region.
    CartesianEndEffector { region_id: u8, byte_offset: u32, byte_len: u32 },
}

/// §1.3.3: derive the Channel B **command** layout from `RobotProfile`,
/// selected regions, and quantization tier -- never transmitted as an
/// explicit register table, same principle as `derive_layout`. Regions are
/// walked in profile order; a `VelocityAttitude`-tagged region only emits
/// a block the first time that shape is encountered among the selected
/// regions, since all such regions share one block.
pub fn derive_command_layout(profile: &RobotProfile, region_ids: &[u8], tier: QuantizationTier) -> Vec<CommandBlock> {
    let mut blocks = Vec::new();
    let mut cursor: u32 = 0;
    let selected = |id: u8| region_ids.is_empty() || region_ids.contains(&id);
    let mut velocity_attitude_emitted = false;

    for region in &profile.regions {
        if !selected(region.region_id) {
            continue;
        }
        match region.command_shape {
            CommandShape::Kinematic => {
                let mut joint_offsets = Vec::new();
                for (i, joint) in profile.joints.iter().enumerate() {
                    if joint.region_id != region.region_id {
                        continue;
                    }
                    let byte_len = tier.bytes_per_field();
                    joint_offsets.push(FieldOffset { joint_index: i as u16, byte_offset: cursor, byte_len });
                    cursor += byte_len;
                }
                blocks.push(CommandBlock::Kinematic { region_id: region.region_id, joint_offsets });
            }
            CommandShape::VelocityAttitude => {
                if velocity_attitude_emitted {
                    continue;
                }
                let byte_len = CommandShape::VelocityAttitude.fixed_field_count().unwrap() * tier.bytes_per_field();
                blocks.push(CommandBlock::VelocityAttitude { byte_offset: cursor, byte_len });
                cursor += byte_len;
                velocity_attitude_emitted = true;
            }
            CommandShape::CartesianEndEffector => {
                let byte_len = CommandShape::CartesianEndEffector.fixed_field_count().unwrap() * tier.bytes_per_field();
                blocks.push(CommandBlock::CartesianEndEffector { region_id: region.region_id, byte_offset: cursor, byte_len });
                cursor += byte_len;
            }
        }
    }
    blocks
}

/// Shared body-frame velocity/attitude command (`CommandShape::VelocityAttitude`,
/// FR-1.5). One robot has at most one of these per tick, regardless of how
/// many regions are tagged with this shape -- lives here, not duplicated
/// per integrator, so every robot using this shape packs/unpacks it the
/// same way.
///
/// Packs every field as one tier-width slot (`QuantizationTier::bytes_per_field`),
/// not the previous hand-tuned mixed i16/u8 layout -- a few bytes larger at
/// Standard tier, traded for a derivable, standardized layout instead of a
/// bespoke one per robot. All three tiers are supported (`pack`/`unpack`);
/// see the tier-scale note on `CONTINUOUS_SCALE_STANDARD`/`_COMPACT` for how
/// Compact trades precision for the range a signed byte allows.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityAttitudeCommand {
    pub vx: f32,
    pub vy: f32,
    pub turn: f32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

/// Standard-tier scale for vx/vy/turn/roll/pitch/yaw (int16, 0.01-unit/LSB,
/// +/-327.67 unit range) -- unchanged from the original Standard-only impl.
const CONTINUOUS_SCALE_STANDARD: f32 = 100.0;
/// Compact-tier scale for the same fields (int8, 1-unit/LSB, +/-127 unit
/// range). `CONTINUOUS_SCALE_STANDARD`'s 0.01-unit/LSB would only reach
/// +/-1.27 units in a signed byte -- nowhere near XGO's own turn-rate
/// commands (up to 60 units, see `input.rs`) -- so Compact deliberately
/// uses a coarser scale to keep the range usable rather than the precision
/// high. This matches the *original* pre-FR-1.5 hand-rolled encoding's
/// scale for vx/vy/turn (it never scaled those, i.e. implicitly 1-unit/LSB).
const CONTINUOUS_SCALE_COMPACT: f32 = 1.0;

fn continuous_scale(tier: QuantizationTier) -> f32 {
    match tier {
        QuantizationTier::Compact => CONTINUOUS_SCALE_COMPACT,
        QuantizationTier::Standard => CONTINUOUS_SCALE_STANDARD,
        QuantizationTier::Full => 1.0, // ignored -- Full stores the exact f32, no scale needed
    }
}

/// Pack one `f32` field at `tier`'s width, scaled by `scale` for the
/// integer tiers (ignored for `Full`, which stores the value exactly).
fn pack_scaled(out: &mut Vec<u8>, value: f32, scale: f32, tier: QuantizationTier) {
    match tier {
        QuantizationTier::Compact => {
            let q = (value * scale).round().clamp(i8::MIN as f32, i8::MAX as f32) as i8;
            out.push(q as u8);
        }
        QuantizationTier::Standard => {
            let q = (value * scale).round().clamp(i16::MIN as f32, i16::MAX as f32) as i16;
            out.extend_from_slice(&q.to_be_bytes());
        }
        QuantizationTier::Full => out.extend_from_slice(&value.to_be_bytes()),
    }
}

/// Inverse of `pack_scaled`. `field` must be exactly `tier.bytes_per_field()` long.
fn unpack_scaled(field: &[u8], scale: f32, tier: QuantizationTier) -> Option<f32> {
    Some(match tier {
        QuantizationTier::Compact => (*field.first()? as i8) as f32 / scale,
        QuantizationTier::Standard => i16::from_be_bytes(field.try_into().ok()?) as f32 / scale,
        QuantizationTier::Full => f32::from_be_bytes(field.try_into().ok()?),
    })
}

/// Pack a `u8`-domain field (e.g. `CartesianCommand.gripper`: already
/// 0-255, i.e. exactly one byte's worth of information) at `tier`'s width.
/// Unlike `pack_scaled`, no scale/clamping is needed at any tier --
/// Compact's 1-byte slot already holds the full 0-255 range exactly.
fn pack_u8(out: &mut Vec<u8>, value: u8, tier: QuantizationTier) {
    match tier {
        QuantizationTier::Compact => out.push(value),
        QuantizationTier::Standard => out.extend_from_slice(&(value as i16).to_be_bytes()),
        QuantizationTier::Full => out.extend_from_slice(&(value as f32).to_be_bytes()),
    }
}

fn unpack_u8(field: &[u8], tier: QuantizationTier) -> Option<u8> {
    Some(match tier {
        QuantizationTier::Compact => *field.first()?,
        QuantizationTier::Standard => i16::from_be_bytes(field.try_into().ok()?) as u8,
        QuantizationTier::Full => f32::from_be_bytes(field.try_into().ok()?).round() as u8,
    })
}

impl VelocityAttitudeCommand {
    pub fn pack(&self, tier: QuantizationTier) -> Vec<u8> {
        let scale = continuous_scale(tier);
        let field_len = tier.bytes_per_field() as usize;
        let mut out = Vec::with_capacity(6 * field_len);
        for v in [self.vx, self.vy, self.turn, self.roll, self.pitch, self.yaw] {
            pack_scaled(&mut out, v, scale, tier);
        }
        out
    }

    pub fn unpack(bytes: &[u8], tier: QuantizationTier) -> Option<Self> {
        let field_len = tier.bytes_per_field() as usize;
        if bytes.len() < 6 * field_len {
            return None;
        }
        let scale = continuous_scale(tier);
        let mut fields = bytes.chunks_exact(field_len);
        Some(Self {
            vx: unpack_scaled(fields.next()?, scale, tier)?,
            vy: unpack_scaled(fields.next()?, scale, tier)?,
            turn: unpack_scaled(fields.next()?, scale, tier)?,
            roll: unpack_scaled(fields.next()?, scale, tier)?,
            pitch: unpack_scaled(fields.next()?, scale, tier)?,
            yaw: unpack_scaled(fields.next()?, scale, tier)?,
        })
    }

    /// Standard-tier convenience wrapper -- the tier this robot's `SESSION_ACCEPT`
    /// currently always selects (`operator-console::session_handler`).
    pub fn pack_standard(&self) -> Vec<u8> {
        self.pack(QuantizationTier::Standard)
    }

    pub fn unpack_standard(bytes: &[u8]) -> Option<Self> {
        Self::unpack(bytes, QuantizationTier::Standard)
    }
}

/// Cartesian end-effector command (`CommandShape::CartesianEndEffector`,
/// FR-1.5). One per region using this shape (a dual-arm robot has two,
/// independently). `y` is unused (always 0) for a robot whose mechanism
/// only has 2 controllable axes, e.g. XGO's `arm(x, z)`, rather than the
/// wire format varying per robot (`DESIGN.md` §1.3.2).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianCommand {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    /// Gripper position, 0-255.
    pub gripper: u8,
}

/// Compact-tier position scale (mm/LSB). XGO's own arm reaches +/-155mm
/// (`xgo_profile.rs`), which doesn't fit in a signed byte at 1mm/LSB
/// (+/-127mm); halving precision to 2mm/LSB (+/-254mm) keeps this robot's
/// own real range representable instead of silently clipping it.
const POSITION_SCALE_COMPACT: f32 = 0.5;

impl CartesianCommand {
    pub fn pack(&self, tier: QuantizationTier) -> Vec<u8> {
        let scale = match tier {
            QuantizationTier::Compact => POSITION_SCALE_COMPACT,
            QuantizationTier::Standard | QuantizationTier::Full => 1.0,
        };
        let field_len = tier.bytes_per_field() as usize;
        let mut out = Vec::with_capacity(4 * field_len);
        for v in [self.x, self.y, self.z] {
            pack_scaled(&mut out, v, scale, tier);
        }
        pack_u8(&mut out, self.gripper, tier);
        out
    }

    pub fn unpack(bytes: &[u8], tier: QuantizationTier) -> Option<Self> {
        let field_len = tier.bytes_per_field() as usize;
        if bytes.len() < 4 * field_len {
            return None;
        }
        let scale = match tier {
            QuantizationTier::Compact => POSITION_SCALE_COMPACT,
            QuantizationTier::Standard | QuantizationTier::Full => 1.0,
        };
        let mut fields = bytes.chunks_exact(field_len);
        Some(Self {
            x: unpack_scaled(fields.next()?, scale, tier)?,
            y: unpack_scaled(fields.next()?, scale, tier)?,
            z: unpack_scaled(fields.next()?, scale, tier)?,
            gripper: unpack_u8(fields.next()?, tier)?,
        })
    }

    /// Standard-tier convenience wrapper -- see `VelocityAttitudeCommand`'s.
    pub fn pack_standard(&self) -> Vec<u8> {
        self.pack(QuantizationTier::Standard)
    }

    pub fn unpack_standard(bytes: &[u8]) -> Option<Self> {
        Self::unpack(bytes, QuantizationTier::Standard)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_profile() -> RobotProfile {
        // XGO-Lite V2 shape: 12 leg joints across 4 regions.
        let mut joints = Vec::new();
        for region in 0..4u8 {
            for _ in 0..3 {
                joints.push(JointDescriptor {
                    min_limit: -65.0,
                    max_limit: 93.0,
                    max_velocity: 300.0,
                    has_torque_sensing: false,
                    region_id: region,
                });
            }
        }
        RobotProfile {
            dof_count: 12,
            joints,
            regions: (0..4u8)
                .map(|r| BodyRegionDescriptor {
                    region_id: r,
                    name: format!("leg_{r}"),
                    joint_start: r as u16 * 3,
                    joint_count: 3,
                    has_force_torque_sensor: false,
                    command_shape: CommandShape::Kinematic,
                })
                .collect(),
            base_type: BaseType::Stationary,
        }
    }

    /// Real XGO-Lite V2 shape (§1.3.2, xgo_profile.rs): 4 `VelocityAttitude`
    /// leg regions sharing one command block, plus one `CartesianEndEffector`
    /// arm region -- the hybrid case FR-1.5 exists for.
    fn xgo_like_profile() -> RobotProfile {
        let mut profile = sample_profile();
        for region in &mut profile.regions {
            region.command_shape = CommandShape::VelocityAttitude;
        }
        profile.joints.extend((0..3).map(|_| JointDescriptor {
            min_limit: -90.0,
            max_limit: 90.0,
            max_velocity: 600.0,
            has_torque_sensing: false,
            region_id: 4,
        }));
        profile.dof_count = 15;
        profile.regions.push(BodyRegionDescriptor {
            region_id: 4,
            name: "arm".to_string(),
            joint_start: 12,
            joint_count: 3,
            has_force_torque_sensor: false,
            command_shape: CommandShape::CartesianEndEffector,
        });
        profile.base_type = BaseType::QuadrupedLegs;
        profile
    }

    #[test]
    fn layout_is_deterministic_given_the_same_inputs() {
        let profile = sample_profile();
        let a = derive_layout(&profile, &[], QuantizationTier::Standard);
        let b = derive_layout(&profile, &[], QuantizationTier::Standard);
        assert_eq!(a, b);
        assert_eq!(a.len(), 12);
        assert_eq!(a[0].byte_offset, 0);
        assert_eq!(a[1].byte_offset, 2); // Standard tier = 2B/field
        assert_eq!(a[11].byte_offset, 22);
    }

    #[test]
    fn region_filter_only_includes_selected_regions_in_wire_order() {
        let profile = sample_profile();
        let layout = derive_layout(&profile, &[0, 2], QuantizationTier::Full);
        assert_eq!(layout.len(), 6); // regions 0 and 2, 3 joints each
        assert_eq!(layout[0].joint_index, 0); // first joint of region 0
        assert_eq!(layout[3].joint_index, 6); // first joint of region 2
    }

    #[test]
    fn profile_hash_is_stable_and_change_sensitive() {
        let profile = sample_profile();
        let h1 = profile.profile_hash();
        let h2 = sample_profile().profile_hash();
        assert_eq!(h1, h2, "identical profiles must hash identically for SESSION_ACCEPT caching");

        let mut mutated = sample_profile();
        mutated.joints[0].max_limit += 1.0;
        assert_ne!(h1, mutated.profile_hash(), "changed joint limits must invalidate the cache");
    }

    #[test]
    fn command_shape_change_invalidates_the_profile_hash() {
        let kinematic = sample_profile();
        let mut hybrid = sample_profile();
        hybrid.regions[0].command_shape = CommandShape::VelocityAttitude;
        assert_ne!(kinematic.profile_hash(), hybrid.profile_hash());
    }

    #[test]
    fn velocity_attitude_regions_collapse_into_one_shared_block() {
        let profile = xgo_like_profile();
        let blocks = derive_command_layout(&profile, &[], QuantizationTier::Standard);

        let va_blocks: Vec<_> = blocks
            .iter()
            .filter(|b| matches!(b, CommandBlock::VelocityAttitude { .. }))
            .collect();
        assert_eq!(va_blocks.len(), 1, "4 VelocityAttitude leg regions must collapse into exactly one block");

        let cartesian_blocks: Vec<_> = blocks
            .iter()
            .filter(|b| matches!(b, CommandBlock::CartesianEndEffector { .. }))
            .collect();
        assert_eq!(cartesian_blocks.len(), 1, "the arm region gets its own independent block");
    }

    #[test]
    fn command_layout_is_deterministic_and_non_overlapping() {
        let profile = xgo_like_profile();
        let a = derive_command_layout(&profile, &[], QuantizationTier::Standard);
        let b = derive_command_layout(&profile, &[], QuantizationTier::Standard);
        assert_eq!(a, b);

        // VelocityAttitude block first (region 0 is the first VelocityAttitude
        // region encountered), then the arm's CartesianEndEffector block
        // immediately after it with no gap or overlap.
        let CommandBlock::VelocityAttitude { byte_offset: va_off, byte_len: va_len } = a[0] else {
            panic!("expected VelocityAttitude block first, got {:?}", a[0]);
        };
        assert_eq!(va_off, 0);
        assert_eq!(va_len, 6 * QuantizationTier::Standard.bytes_per_field());

        let CommandBlock::CartesianEndEffector { byte_offset: c_off, byte_len: c_len, .. } = a[1] else {
            panic!("expected CartesianEndEffector block second, got {:?}", a[1]);
        };
        assert_eq!(c_off, va_off + va_len, "no gap/overlap between blocks");
        assert_eq!(c_len, 4 * QuantizationTier::Standard.bytes_per_field());
    }

    #[test]
    fn velocity_attitude_command_round_trips() {
        let cmd = VelocityAttitudeCommand { vx: 15.0, vy: -12.0, turn: 60.0, roll: 1.5, pitch: -2.25, yaw: 0.0 };
        let bytes = cmd.pack_standard();
        assert_eq!(bytes.len(), 12);
        let round_tripped = VelocityAttitudeCommand::unpack_standard(&bytes).unwrap();
        assert_eq!(round_tripped, cmd);
    }

    #[test]
    fn cartesian_command_round_trips_with_unused_axis_zeroed() {
        // XGO's arm is 2-axis (x, z via xgolib's arm(x, z)) -- y stays 0
        // rather than the wire format varying per robot (DESIGN.md §1.3.2).
        let cmd = CartesianCommand { x: 40.0, y: 0.0, z: -10.0, gripper: 200 };
        let bytes = cmd.pack_standard();
        assert_eq!(bytes.len(), 8);
        let round_tripped = CartesianCommand::unpack_standard(&bytes).unwrap();
        assert_eq!(round_tripped, cmd);
    }

    #[test]
    fn velocity_attitude_command_round_trips_at_every_tier() {
        let cmd = VelocityAttitudeCommand { vx: 15.0, vy: -12.0, turn: 60.0, roll: 1.5, pitch: -2.25, yaw: 0.0 };

        let full = cmd.pack(QuantizationTier::Full);
        assert_eq!(full.len(), 6 * 4);
        assert_eq!(VelocityAttitudeCommand::unpack(&full, QuantizationTier::Full).unwrap(), cmd, "Full tier is exact, no quantization error");

        let compact = cmd.pack(QuantizationTier::Compact);
        assert_eq!(compact.len(), 6);
        let round_tripped = VelocityAttitudeCommand::unpack(&compact, QuantizationTier::Compact).unwrap();
        // Compact is 1-unit/LSB (no attitude sub-degree precision), so
        // integer-valued fields (vx/vy/turn here) round-trip exactly and
        // fractional attitude values (roll/pitch) are within 1 LSB.
        assert_eq!(round_tripped.vx, 15.0);
        assert_eq!(round_tripped.vy, -12.0);
        assert_eq!(round_tripped.turn, 60.0);
        assert!((round_tripped.roll - 1.5).abs() <= 1.0);
        assert!((round_tripped.pitch - (-2.25)).abs() <= 1.0);
    }

    #[test]
    fn velocity_attitude_compact_tier_does_not_clip_a_real_turn_command() {
        // A 60-unit turn command is real (input.rs's Left/Right turn keys).
        // At CONTINUOUS_SCALE_STANDARD's 0.01-unit/LSB this would clip hard
        // in a signed byte; Compact's own scale must not repeat that.
        let cmd = VelocityAttitudeCommand { vx: 0.0, vy: 0.0, turn: 60.0, roll: 0.0, pitch: 0.0, yaw: 0.0 };
        let bytes = cmd.pack(QuantizationTier::Compact);
        let round_tripped = VelocityAttitudeCommand::unpack(&bytes, QuantizationTier::Compact).unwrap();
        assert_eq!(round_tripped.turn, 60.0, "a real, in-range command must not silently clip");
    }

    #[test]
    fn cartesian_command_round_trips_at_every_tier() {
        let cmd = CartesianCommand { x: 40.0, y: 0.0, z: -10.0, gripper: 200 };

        let full = cmd.pack(QuantizationTier::Full);
        assert_eq!(full.len(), 4 * 4);
        assert_eq!(CartesianCommand::unpack(&full, QuantizationTier::Full).unwrap(), cmd);

        let compact = cmd.pack(QuantizationTier::Compact);
        assert_eq!(compact.len(), 4);
        let round_tripped = CartesianCommand::unpack(&compact, QuantizationTier::Compact).unwrap();
        assert!((round_tripped.x - 40.0).abs() <= 2.0, "Compact position precision is 2mm/LSB");
        assert!((round_tripped.z - (-10.0)).abs() <= 2.0);
        assert_eq!(round_tripped.gripper, 200, "gripper is always exact, every tier");
    }

    #[test]
    fn cartesian_compact_tier_does_not_clip_this_robots_own_arm_reach() {
        // xgo_profile.rs's real arm range is x:[-80,155], z:[-95,155] --
        // 155mm doesn't fit in a signed byte at 1mm/LSB (+/-127), which is
        // exactly why POSITION_SCALE_COMPACT halves precision instead.
        let cmd = CartesianCommand { x: 155.0, y: 0.0, z: -95.0, gripper: 255 };
        let bytes = cmd.pack(QuantizationTier::Compact);
        let round_tripped = CartesianCommand::unpack(&bytes, QuantizationTier::Compact).unwrap();
        assert!((round_tripped.x - 155.0).abs() <= 2.0, "this robot's own max reach must not clip at Compact tier");
        assert!((round_tripped.z - (-95.0)).abs() <= 2.0);
        assert_eq!(round_tripped.gripper, 255);
    }

    #[test]
    fn standard_tier_wrapper_is_byte_identical_to_pack_with_standard_tier() {
        let cmd = VelocityAttitudeCommand { vx: 1.0, vy: 2.0, turn: 3.0, roll: 4.0, pitch: 5.0, yaw: 6.0 };
        assert_eq!(cmd.pack_standard(), cmd.pack(QuantizationTier::Standard));

        let arm = CartesianCommand { x: 1.0, y: 2.0, z: 3.0, gripper: 4 };
        assert_eq!(arm.pack_standard(), arm.pack(QuantizationTier::Standard));
    }

    #[test]
    fn sr1_applies_only_to_biped_legs() {
        // FR-1.6.1: dynamically-balanced bipeds are the one morphology
        // where the WBC/ZMP override is load-bearing; everything else,
        // including the not-yet-classified `Other`, is a documented no-op.
        assert!(BaseType::BipedLegs.requires_dynamic_balance_override());
        for other in [
            BaseType::Stationary,
            BaseType::WheeledStandard,
            BaseType::WheeledHolonomic,
            BaseType::QuadrupedLegs,
            BaseType::Other,
        ] {
            assert!(!other.requires_dynamic_balance_override(), "{other:?} must not require SR-1's balance override");
        }
    }

    #[test]
    fn only_wheeled_standard_has_no_meaningful_lateral_velocity() {
        assert!(!BaseType::WheeledStandard.lateral_velocity_meaningful());
        for other in [
            BaseType::Stationary,
            BaseType::WheeledHolonomic,
            BaseType::BipedLegs,
            BaseType::QuadrupedLegs,
            BaseType::Other,
        ] {
            assert!(other.lateral_velocity_meaningful(), "{other:?} should not be treated as unable to strafe");
        }
    }

    #[test]
    fn base_type_change_invalidates_the_profile_hash() {
        let mut wheeled = sample_profile();
        wheeled.base_type = BaseType::WheeledStandard;
        let mut biped = sample_profile();
        biped.base_type = BaseType::BipedLegs;
        assert_ne!(wheeled.profile_hash(), biped.profile_hash());
    }
}
