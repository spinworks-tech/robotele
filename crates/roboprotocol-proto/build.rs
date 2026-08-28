use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let schema = "schemas/roboprotocol.fbs";
    println!("cargo:rerun-if-changed={schema}");

    let out_dir = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR set by cargo"));

    let status = Command::new("flatc")
        .args(["--rust", "-o"])
        .arg(&out_dir)
        .arg(schema)
        .status();

    match status {
        Ok(s) if s.success() => {}
        Ok(s) => panic!("flatc exited with status {s} compiling {schema}"),
        Err(e) => panic!(
            "failed to run `flatc` (needed to generate FlatBuffers Rust bindings for {schema}): {e}\n\n\
             Install the FlatBuffers compiler and retry:\n  \
             Debian/Ubuntu/Raspberry Pi OS: sudo apt install flatbuffers-compiler\n\
             (this is also a documented prerequisite for building this workspace on the CM4 --\n\
             see the plan's CM4 setup checklist)."
        ),
    }

    // flatc names the output file after the schema's basename.
    let generated = out_dir.join("roboprotocol_generated.rs");
    assert!(
        generated.exists(),
        "flatc ran but did not produce the expected {generated:?} -- check the flatc version's \
         output naming convention (`flatc --version`)"
    );
}
