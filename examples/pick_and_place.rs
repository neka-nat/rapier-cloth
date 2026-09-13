//! Grasp, lift, transport and release a cloth; optionally write a JSON recording.
//! Run: cargo run --release --example pick_and_place -- --help
#[path = "support/recording.rs"]
mod recording;
use std::{fs, path::Path};
fn write_json(path: &str, value: &impl serde::Serialize) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(parent) = Path::new(path)
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
    {
        fs::create_dir_all(parent)?;
    }
    let file = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)?;
    serde_json::to_writer(std::io::BufWriter::new(file), value)?;
    Ok(())
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let mut record = None;
    let mut summary_path = None;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--record" => record = Some(args.next().ok_or("--record requires a path")?),
            "--summary" => summary_path = Some(args.next().ok_or("--summary requires a path")?),
            "--help" => {
                println!("pick_and_place [--record NEW_FILE.json] [--summary NEW_FILE.json]");
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    let (recording, summary) = recording::simulate(Default::default())?;
    if let Some(path) = record {
        write_json(&path, &recording)?;
    }
    if let Some(path) = summary_path {
        write_json(&path, &summary)?;
    }
    println!("{}", serde_json::to_string_pretty(&summary)?);
    Ok(())
}
