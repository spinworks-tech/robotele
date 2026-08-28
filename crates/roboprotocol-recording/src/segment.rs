//! Segment file naming, writing, and directory listing for one category.
//! Filenames are the zero-padded start `capture_us`, so lexicographic
//! order is chronological order -- no separate index needed to find the
//! oldest/newest segment for rotation or retention.

use std::fs::{self, File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;

use roboprotocol_core::recording::{encode_record, RecordHeader, SegmentMeta};

pub struct OpenSegment {
    writer: BufWriter<File>,
    pub size_bytes: u64,
    pub opened_at: Instant,
}

impl OpenSegment {
    pub fn create(dir: &Path, start_capture_us: u64) -> std::io::Result<Self> {
        fs::create_dir_all(dir)?;
        let path = dir.join(format!("{start_capture_us:020}.rec"));
        let file = OpenOptions::new().create(true).append(true).open(path)?;
        Ok(Self { writer: BufWriter::new(file), size_bytes: 0, opened_at: Instant::now() })
    }

    pub fn write_record(&mut self, header: RecordHeader, payload: &[u8]) -> std::io::Result<()> {
        let mut buf = Vec::with_capacity(13 + payload.len());
        encode_record(header, payload, &mut buf);
        self.writer.write_all(&buf)?;
        self.size_bytes += buf.len() as u64;
        Ok(())
    }

    pub fn flush(&mut self) -> std::io::Result<()> {
        self.writer.flush()?;
        self.writer.get_ref().sync_data()
    }
}

/// Existing segments in `dir`, oldest-first. Empty (not an error) if the
/// category directory doesn't exist yet -- the common case before the
/// first segment is ever created.
pub fn list_segments(dir: &Path) -> std::io::Result<Vec<(PathBuf, SegmentMeta)>> {
    if !dir.exists() {
        return Ok(Vec::new());
    }
    let mut entries: Vec<(PathBuf, SegmentMeta)> = fs::read_dir(dir)?
        .filter_map(|e| e.ok())
        .filter(|e| e.path().extension().is_some_and(|ext| ext == "rec"))
        .filter_map(|e| {
            let meta = e.metadata().ok()?;
            Some((e.path(), SegmentMeta { size_bytes: meta.len() }))
        })
        .collect();
    entries.sort_by(|a, b| a.0.cmp(&b.0));
    Ok(entries)
}
