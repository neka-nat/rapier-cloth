#!/usr/bin/env python3
"""Exercise the actual fold_towel CLI; a nine-step smoke is not fold qualification."""
import argparse
import json
import math
from pathlib import Path
import subprocess


def check(binary, precision, directory, checkout):
    binary = binary.resolve(strict=True)
    directory.mkdir(parents=True, exist_ok=False)
    directory = directory.resolve()

    def invoke(name, args, expected):
        result = subprocess.run([str(binary), *args], cwd=directory,
                                capture_output=True, text=True, timeout=180)
        (directory / f"{name}.stdout").write_text(result.stdout)
        (directory / f"{name}.stderr").write_text(result.stderr)
        assert result.returncode == expected, (name, result.returncode, result.stderr)
        return result.stdout

    assert "--max-steps" in invoke("help", ["--help"], 0)
    for name, args in [
        ("invalid-variant", ["--variant", "6"]),
        ("zero-limit", ["--max-steps", "0"]),
        ("oversized-limit", ["--max-steps", "3601"]),
        ("missing-record-path", ["--record"]),
    ]:
        invoke(name, args, 1)
    sentinel = directory / "existing.json"
    sentinel.write_text("existing user recording\n")
    invoke("existing-output", ["--record", str(sentinel), "--max-steps", "9"], 1)
    assert sentinel.read_text() == "existing user recording\n"

    stdout = invoke("partial", ["--max-steps", "9", "--record", "recording.json",
                                "--summary", "summary.json"], 2)
    summary = json.loads((directory / "summary.json").read_text())
    assert summary == json.loads(stdout)
    assert summary["schema_version"] == 2 and summary["fixture_version"] == 2
    assert summary["precision"] == precision and summary["recording_enabled"]
    assert summary["warmup_substeps"] == 120 and summary["substeps_per_frame"] == 4
    if checkout:
        commit = subprocess.check_output(["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True).strip()
        dirty = bool(subprocess.check_output(["git", "-C", str(checkout), "status", "--porcelain"], text=True).strip())
        assert (summary["commit"], summary["dirty"]) == (commit, dirty)
    else:
        assert summary["commit"] == "unknown" and summary["dirty"] is True
    run, = summary["runs"]
    assert run["stop_reason"] == "step_limit" and run["steps"] == 9
    assert run["failure"] is None and run["failure_next_step"] is None
    assert run["finite"] and run["diagnostics_finite"] and run["audited_substeps"] == 9
    assert run["verification_enabled"] and run["physics_settings"]["iterations"] == 8
    assert math.isclose(run["physics_settings"]["h"], 1 / 240, rel_tol=1e-6)
    assert run["mesh"]["vertices"] == 1024 and run["mesh"]["triangles"] == 1922
    for flag in ["self_collision", "continuous_self_collision", "rigid_surface_collision", "continuous_rigid_collision"]:
        assert run["collision_settings"][flag] is True
    assert len(run["physics_samples_ms"]) == 2  # Partial frame is not a timed frame.

    record = json.loads((directory / "recording.json").read_text())
    assert record["schema_version"] == 1 and record["precision"] == precision
    assert record["config"] == run["config"]
    assert record["outcome"] == {"stop_reason": "step_limit", "steps": 9,
                                 "end_step": 3600, "failure": None}
    assert [f["step"] for f in record["frames"]] == [0, 4, 8, 9]
    assert [s["id"] for s in record["shapes"]] == [0, 1, 2]
    assert len(record["triangles"]) == 1922
    assert record["frames"][0]["positions"] != record["frames"][-1]["positions"]
    for frame in record["frames"]:
        assert [b["id"] for b in frame["bodies"]] == [0, 1, 2]
        assert len(frame["positions"]) == 1024
        assert math.isclose(frame["time"], frame["step"] / 240, abs_tol=1e-8)
        assert all(math.isfinite(x) for p in frame["positions"] for x in p)
        assert frame["attached_particles"] == frame["anchors"] == frame["pinned_particles"] == []
    print(f"Verified {precision} fold_towel CLI and 9-step replay; full folding remains unqualified: {directory}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--precision", choices=["f32", "f64"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--checkout", type=Path)
    args = parser.parse_args()
    check(args.binary, args.precision, args.output, args.checkout)
