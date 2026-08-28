//! The dedicated writer thread body for one category: drains its ring,
//! rotates/enforces retention, writes, and flushes on a schedule
//! (DESIGN.md §10.3).

use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Instant;

use roboprotocol_core::recording::{self, RecordHeader, RotationThresholds};

use crate::ring::CategoryState;
use crate::segment::{self, OpenSegment};
use crate::CategoryConfig;

/// How often the writer wakes even with nothing queued, so periodic
/// flush/rotation checks still happen during a lull rather than only
/// when a new record arrives.
const POLL_INTERVAL: std::time::Duration = std::time::Duration::from_millis(200);

pub fn run(dir: PathBuf, config: CategoryConfig, state: Arc<CategoryState>) {
    let thresholds = RotationThresholds { max_segment_bytes: config.max_segment_bytes, max_segment_duration: config.max_segment_duration };

    let mut segment: Option<OpenSegment> = None;
    let mut last_flush = Instant::now();

    loop {
        let batch = state.drain(POLL_INTERVAL);
        let shutting_down = state.shutdown.load(Ordering::Relaxed);
        if shutting_down && batch.is_empty() {
            break;
        }

        for record in batch {
            // No re-check of `state.active` here: `Recorder::enqueue`
            // already gated this record's entry into the ring on being
            // active *at enqueue time*, which is the correct moment to
            // decide -- re-checking here, against *current* state, races
            // the writer being behind real-time against the record that
            // itself turns recording off (or any record enqueued in the
            // same window). A record that made it into the ring earned
            // its place there and gets written, regardless of what the
            // category's active flag says by the time this drains.
            let needs_rotate = match &segment {
                Some(seg) => recording::should_rotate(seg.size_bytes, seg.opened_at, Instant::now(), &thresholds),
                None => true,
            };
            if needs_rotate {
                if let Some(mut seg) = segment.take() {
                    let _ = seg.flush();
                }
                enforce_retention(&dir, record.payload.len() as u64, config.max_total_bytes);
                segment = OpenSegment::create(&dir, record.capture_us).ok();
            }
            if let Some(seg) = &mut segment {
                let header = RecordHeader { capture_us: record.capture_us, control_source: record.control_source };
                let _ = seg.write_record(header, &record.payload);
            }
        }

        if last_flush.elapsed() >= config.flush_interval {
            if let Some(seg) = &mut segment {
                let _ = seg.flush();
            }
            last_flush = Instant::now();
        }
    }

    if let Some(mut seg) = segment {
        let _ = seg.flush();
    }
}

fn enforce_retention(dir: &Path, incoming_bytes: u64, budget_bytes: u64) {
    let Ok(existing) = segment::list_segments(dir) else { return };
    let metas: Vec<_> = existing.iter().map(|(_, m)| *m).collect();
    let to_delete = recording::segments_to_delete_for_budget(&metas, incoming_bytes, budget_bytes);
    for idx in to_delete {
        let _ = std::fs::remove_file(&existing[idx].0);
    }
}
