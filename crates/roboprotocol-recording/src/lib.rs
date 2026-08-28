//! Local recording writer pipeline (DESIGN.md §10, REQUIREMENTS.md FR-9).
//! Bounded per-category ring buffers feeding dedicated writer threads,
//! decoupled from whatever real-time control loop is producing the data
//! (§10.3) -- plain `std::thread`/`std::sync`, no async runtime, so this
//! crate is usable identically from either binary regardless of what
//! their own event loops look like, and needs no new `tokio` features on
//! either.
//!
//! Pure record framing and the rotation/retention *decisions* live in
//! `roboprotocol_core::recording`; this crate is only the actual
//! thread/file-I/O plumbing around them.
//!
//! Recorded video/`ActionTrigger` payloads are the §10.2-framed bytes
//! (header + unmodified wire payload), not a raw elementary stream a
//! player can open directly -- §10.2 explicitly accepts this trade-off
//! ("a segment file is read by exactly one thing... in the same
//! codebase"), namely `tools/replay/recording_to_mp4.py`, which strips
//! each record's header and remuxes a `video-a` segment directory
//! straight into an `.mp4` (no re-encode -- the recorded payload is
//! already start-code-prefixed H.264, see `channel_a.rs`'s
//! `spawn_encoder`).

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

mod ring;
mod segment;
mod writer;

use ring::CategoryState;

/// Which wire channel (or, for `KeyPress`, which operator-console-only
/// concept) a record belongs to. `robot-edge` never configures
/// `KeyPress` -- it has no keyboard to log.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Category {
    VideoA,
    ChannelBCommand,
    ChannelBTelemetry,
    ChannelBHaptic,
    ActionTriggerC,
    KeyPress,
}

impl Category {
    /// Segment subdirectory name for this category.
    pub fn dirname(self) -> &'static str {
        match self {
            Category::VideoA => "video-a",
            Category::ChannelBCommand => "channel-b-command",
            Category::ChannelBTelemetry => "channel-b-telemetry",
            Category::ChannelBHaptic => "channel-b-haptic",
            Category::ActionTriggerC => "action-trigger-c",
            Category::KeyPress => "key-press",
        }
    }
}

#[derive(Debug, Clone)]
pub struct CategoryConfig {
    pub category: Category,
    pub ring_capacity: usize,
    pub max_segment_bytes: u64,
    pub max_segment_duration: Duration,
    pub max_total_bytes: u64,
    pub flush_interval: Duration,
    /// If `false`, the category's ring/thread exist but every `enqueue`
    /// is a silent no-op until `Recorder::set_active` turns it on --
    /// distinct from the category being absent from `RecorderConfig`
    /// entirely, which is fully inert (no thread at all).
    pub start_active: bool,
}

#[derive(Debug, Clone, Default)]
pub struct RecorderConfig {
    pub record_dir: PathBuf,
    /// A category absent here has no thread, no ring, no directory --
    /// `enqueue` for it is a no-op that doesn't even look anything up
    /// beyond a hash-map miss.
    pub categories: Vec<CategoryConfig>,
}

/// One record to enqueue. `capture_us` is always *this* endpoint's own
/// local wall-clock time -- see `roboprotocol_core::recording`'s module
/// doc for why that's deliberate, not a simplification.
#[derive(Debug, Clone)]
pub struct Record {
    pub capture_us: u64,
    pub control_source: u8,
    pub payload: Vec<u8>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct CategoryStats {
    pub bytes_enqueued: u64,
    pub records_dropped: u64,
    pub active: bool,
}

struct CategoryHandle {
    state: Arc<CategoryState>,
    thread: Option<std::thread::JoinHandle<()>>,
}

/// Cheap to clone (an `Arc` underneath) -- hand a clone to anything that
/// needs to enqueue records; the writer threads and their state are
/// shared, not duplicated.
#[derive(Clone)]
pub struct Recorder {
    inner: Arc<RecorderInner>,
}

struct RecorderInner {
    categories: HashMap<Category, CategoryHandle>,
}

impl Recorder {
    /// Spawns one writer thread per configured category and returns
    /// immediately.
    pub fn start(config: RecorderConfig) -> std::io::Result<Self> {
        let mut categories = HashMap::new();
        for cat_config in config.categories {
            let category = cat_config.category;
            let state = Arc::new(CategoryState::new(cat_config.ring_capacity, cat_config.start_active));
            let dir = config.record_dir.join(category.dirname());
            let thread_state = state.clone();
            let thread = std::thread::spawn(move || writer::run(dir, cat_config, thread_state));
            categories.insert(category, CategoryHandle { state, thread: Some(thread) });
        }
        Ok(Self { inner: Arc::new(RecorderInner { categories }) })
    }

    /// Non-blocking. A no-op if `category` was never configured, or is
    /// currently inactive -- the entire real-time-path cost of a live
    /// call is one short `Mutex` critical section (NFR-6.1's <=1ms
    /// bound).
    pub fn enqueue(&self, category: Category, record: Record) {
        if let Some(handle) = self.inner.categories.get(&category) {
            if handle.state.active.load(Ordering::Relaxed) {
                handle.state.push(record);
            }
        }
    }

    /// No-op if `category` was never configured in `RecorderConfig`.
    pub fn set_active(&self, category: Category, active: bool) {
        if let Some(handle) = self.inner.categories.get(&category) {
            handle.state.active.store(active, Ordering::Relaxed);
            handle.state.wake();
        }
    }

    pub fn is_active(&self, category: Category) -> bool {
        self.inner.categories.get(&category).map(|h| h.state.active.load(Ordering::Relaxed)).unwrap_or(false)
    }

    pub fn stats(&self, category: Category) -> Option<CategoryStats> {
        self.inner.categories.get(&category).map(|h| {
            let (bytes_enqueued, records_dropped, active) = h.state.stats();
            CategoryStats { bytes_enqueued, records_dropped, active }
        })
    }
}

impl Drop for RecorderInner {
    /// Runs once, when the last `Recorder` clone is dropped (process
    /// exit, or an explicit `drop`) -- signals every writer thread to
    /// finish its current batch, flush, and exit, then joins them so a
    /// dropped `Recorder` never leaves data sitting unflushed in memory.
    fn drop(&mut self) {
        for handle in self.categories.values() {
            handle.state.shutdown.store(true, Ordering::Relaxed);
            handle.state.wake();
        }
        for handle in self.categories.values_mut() {
            if let Some(t) = handle.thread.take() {
                let _ = t.join();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant as StdInstant;

    /// Polls `check` until it returns `true` or `timeout` elapses --
    /// never a fixed `sleep`, so these tests run fast when the condition
    /// is met quickly and only wait as long as actually necessary.
    fn wait_until(timeout: Duration, mut check: impl FnMut() -> bool) -> bool {
        let start = StdInstant::now();
        loop {
            if check() {
                return true;
            }
            if start.elapsed() >= timeout {
                return false;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    fn quick_category(category: Category, start_active: bool) -> CategoryConfig {
        CategoryConfig {
            category,
            ring_capacity: 64,
            max_segment_bytes: 1_000_000,
            max_segment_duration: Duration::from_secs(3600),
            max_total_bytes: 10_000_000,
            flush_interval: Duration::from_millis(20),
            start_active,
        }
    }

    fn count_segment_files(dir: &std::path::Path) -> usize {
        segment::list_segments(dir).map(|v| v.len()).unwrap_or(0)
    }

    #[test]
    fn unconfigured_category_enqueue_is_a_noop() {
        let dir = tempfile::tempdir().unwrap();
        let recorder = Recorder::start(RecorderConfig { record_dir: dir.path().to_path_buf(), categories: vec![] }).unwrap();
        recorder.enqueue(Category::VideoA, Record { capture_us: 1, control_source: 0xFF, payload: vec![1, 2, 3] });
        assert!(recorder.stats(Category::VideoA).is_none());
        assert!(!dir.path().join(Category::VideoA.dirname()).exists());
    }

    #[test]
    fn inactive_category_enqueue_is_a_noop() {
        let dir = tempfile::tempdir().unwrap();
        let recorder = Recorder::start(RecorderConfig {
            record_dir: dir.path().to_path_buf(),
            categories: vec![quick_category(Category::ChannelBCommand, false)],
        })
        .unwrap();
        recorder.enqueue(Category::ChannelBCommand, Record { capture_us: 1, control_source: 3, payload: vec![9, 9] });
        let stats = recorder.stats(Category::ChannelBCommand).unwrap();
        assert_eq!(stats.bytes_enqueued, 0);
        assert!(!stats.active);
    }

    /// Regression test: a record enqueued while active must still be
    /// written even if the category is deactivated immediately
    /// afterward, before the writer thread has necessarily drained it --
    /// e.g. the exact keypress that turns recording off, logged (while
    /// still active) a moment before `on_input` flips it off. Caught on
    /// real hardware: the writer used to re-check *current* active state
    /// per record at write time, racing this exact sequence and silently
    /// dropping the record.
    #[test]
    fn record_enqueued_just_before_deactivation_still_gets_written() {
        let dir = tempfile::tempdir().unwrap();
        let recorder = Recorder::start(RecorderConfig {
            record_dir: dir.path().to_path_buf(),
            categories: vec![quick_category(Category::KeyPress, true)],
        })
        .unwrap();
        recorder.enqueue(Category::KeyPress, Record { capture_us: 1, control_source: 0xFF, payload: b"ToggleRecording".to_vec() });
        recorder.set_active(Category::KeyPress, false); // simulates on_input running right after

        let seg_dir = dir.path().join(Category::KeyPress.dirname());
        assert!(
            wait_until(Duration::from_secs(2), || {
                segment::list_segments(&seg_dir).map(|s| s.iter().any(|(_, m)| m.size_bytes > 0)).unwrap_or(false)
            }),
            "the record enqueued just before deactivation was never written"
        );
        let segments = segment::list_segments(&seg_dir).unwrap();
        let bytes = std::fs::read(&segments[0].0).unwrap();
        let records: Vec<_> = roboprotocol_core::recording::decode_records(&bytes).collect();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].1, b"ToggleRecording");
    }

    #[test]
    fn active_category_writes_a_readable_segment() {
        let dir = tempfile::tempdir().unwrap();
        let recorder = Recorder::start(RecorderConfig {
            record_dir: dir.path().to_path_buf(),
            categories: vec![quick_category(Category::KeyPress, true)],
        })
        .unwrap();
        recorder.enqueue(Category::KeyPress, Record { capture_us: 100, control_source: 0xFF, payload: b"Quit".to_vec() });

        let seg_dir = dir.path().join(Category::KeyPress.dirname());
        // Wait for the flush, not just file *existence* -- the file is
        // created before its buffered content is necessarily on disk.
        assert!(
            wait_until(Duration::from_secs(2), || segment::list_segments(&seg_dir).map(|s| s.iter().any(|(_, m)| m.size_bytes > 0)).unwrap_or(false)),
            "segment was never flushed with content"
        );

        let segments = segment::list_segments(&seg_dir).unwrap();
        let bytes = std::fs::read(&segments[0].0).unwrap();
        let records: Vec<_> = roboprotocol_core::recording::decode_records(&bytes).collect();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].0.capture_us, 100);
        assert_eq!(records[0].1, b"Quit");
    }

    #[test]
    fn rotation_fires_at_a_tiny_size_threshold() {
        let dir = tempfile::tempdir().unwrap();
        let mut config = quick_category(Category::ChannelBTelemetry, true);
        config.max_segment_bytes = 20; // header alone is 13 bytes; forces rotation quickly
        let recorder =
            Recorder::start(RecorderConfig { record_dir: dir.path().to_path_buf(), categories: vec![config] }).unwrap();

        let seg_dir = dir.path().join(Category::ChannelBTelemetry.dirname());
        for i in 0..20u64 {
            recorder.enqueue(Category::ChannelBTelemetry, Record { capture_us: 1000 + i, control_source: 3, payload: vec![0u8; 10] });
        }
        assert!(wait_until(Duration::from_secs(2), || count_segment_files(&seg_dir) >= 2), "rotation never produced a second segment");
    }

    #[test]
    fn retention_deletes_oldest_segments_first_and_keeps_the_newest() {
        let dir = tempfile::tempdir().unwrap();
        let mut config = quick_category(Category::ChannelBCommand, true);
        config.max_segment_bytes = 20; // rotate almost every record
        config.max_total_bytes = 60; // budget for roughly 2 segments' worth
        let recorder =
            Recorder::start(RecorderConfig { record_dir: dir.path().to_path_buf(), categories: vec![config] }).unwrap();

        let seg_dir = dir.path().join(Category::ChannelBCommand.dirname());
        let last_capture_us = 1000 + 29;
        for i in 0..30u64 {
            recorder.enqueue(Category::ChannelBCommand, Record { capture_us: 1000 + i, control_source: 3, payload: vec![0u8; 10] });
        }
        // Wait for the writer to actually finish processing the whole
        // backlog -- not just "a couple of segments exist," which (under
        // CPU contention from other tests running in parallel) can be
        // true long before the writer has caught up, making the
        // assertions below run prematurely mid-catch-up rather than once
        // retention has actually settled. The newest segment's numeric
        // (capture_us) prefix reaching the last enqueued value is a
        // direct signal of "caught up," regardless of how fast/slow the
        // writer thread got scheduled.
        let numeric_prefix = |p: &std::path::Path| -> u64 { p.file_stem().unwrap().to_string_lossy().parse().unwrap() };
        assert!(
            wait_until(Duration::from_secs(5), || {
                segment::list_segments(&seg_dir).map(|s| s.last().is_some_and(|(p, _)| numeric_prefix(p) == last_capture_us)).unwrap_or(false)
            }),
            "writer never caught up to the last enqueued record"
        );
        std::thread::sleep(Duration::from_millis(100)); // let the final segment's periodic flush land

        let segments = segment::list_segments(&seg_dir).unwrap();
        assert!(!segments.is_empty(), "retention deleted everything, including the newest");
        // Every surviving segment's numeric (capture_us) prefix must be
        // from the later half of the 1000..1029 range enqueued -- proves
        // the *oldest* ones were the ones deleted, not an arbitrary
        // subset, without pinning the test to an exact filename.
        for (path, _) in &segments {
            let prefix = numeric_prefix(path);
            assert!(prefix >= 1010, "surviving segment {prefix} looks like one of the oldest, retention should have deleted it");
        }
        // Total on-disk size shouldn't be wildly over budget (some slack
        // for the segment actively being written to when the budget was
        // last checked).
        let total: u64 = segments.iter().map(|(_, m)| m.size_bytes).sum();
        assert!(total < 200, "retention did not keep total size bounded: {total} bytes across {} segments", segments.len());
    }

    #[test]
    fn ring_drops_oldest_when_over_capacity() {
        let state = ring::CategoryState::new(4, true);
        for i in 0..10u64 {
            state.push(Record { capture_us: i, control_source: 0xFF, payload: vec![] });
        }
        let (_, dropped, _) = state.stats();
        assert!(dropped >= 6, "expected at least 6 drops (10 pushed, capacity 4), got {dropped}");
    }

    #[test]
    fn segment_crash_safety_truncated_final_record_preserves_earlier_ones() {
        let dir = tempfile::tempdir().unwrap();
        let mut seg = segment::OpenSegment::create(dir.path(), 1).unwrap();
        seg.write_record(roboprotocol_core::recording::RecordHeader { capture_us: 1, control_source: 0 }, b"first").unwrap();
        seg.write_record(roboprotocol_core::recording::RecordHeader { capture_us: 2, control_source: 0 }, b"second-record-longer").unwrap();
        seg.flush().unwrap();
        drop(seg);

        let segments = segment::list_segments(dir.path()).unwrap();
        assert_eq!(segments.len(), 1);
        let mut bytes = std::fs::read(&segments[0].0).unwrap();
        let cut_at = bytes.len() - 5; // chop the tail of the second record
        bytes.truncate(cut_at);

        let records: Vec<_> = roboprotocol_core::recording::decode_records(&bytes).collect();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].1, b"first");
    }
}
