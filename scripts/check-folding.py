#!/usr/bin/env python3
"""Check recorded folding correctness gates against the repository's frozen fixture.

This checks report content, not solver code, machine isolation or live pacing.
The default checks complete audited runs; --suite requires all six variants in
both precisions from one clean source revision. Timing samples are checked for
consistency, but an interleaved correctness replay cannot qualify CPU performance.
"""
import argparse
import copy
import json
import math
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
FIXTURE = ROOT / "examples/support/fold_fixture_v2.json"
SCOPE = "Rapier, snapshots, task transitions and cloth; excludes oracle, task metrics, rendering and output"
LIMITS = {"candidate_pairs": 2_000_000, "retained_contacts": 65_536, "ccd_checks": 2_000_000}
PHASES = (
    ("Settle", "settle_end"), ("Approach", "attach_step"),
    ("Lift", "lift_end"), ("Fold", "fold_end"), ("Lower", "lower_end"),
    ("Hold", "release_step"), ("Retract", "retract_end"), ("Released", "end_step"),
)


class InvalidReport(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise InvalidReport(message)


def number(value, label):
    require(type(value) in (int, float), f"{label}: expected a number")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    require(finite and value >= 0, f"{label}: expected a finite nonnegative number")
    return value


def count(value, label):
    require(type(value) is int and value >= 0, f"{label}: expected a nonnegative integer")
    return value


def same(actual, expected, label, relative=1e-13, absolute=0.0):
    if isinstance(expected, dict):
        require(type(actual) is dict and actual.keys() == expected.keys(), f"{label}: wrong fields")
        for key in expected:
            same(actual[key], expected[key], f"{label}.{key}", relative, absolute)
    elif isinstance(expected, list):
        require(type(actual) is list and len(actual) == len(expected), f"{label}: wrong list length")
        for index, (a, e) in enumerate(zip(actual, expected)):
            same(a, e, f"{label}[{index}]", relative, absolute)
    elif type(expected) is float:
        require(type(actual) in (int, float) and math.isfinite(actual), f"{label}: expected finite numeric value")
        require(math.isclose(actual, expected, rel_tol=relative, abs_tol=absolute), f"{label}: differs from expected {expected}")
    else:
        require(type(actual) is type(expected) and actual == expected, f"{label}: expected {expected!r}")


def finite_tree(value, label="report"):
    if type(value) is float:
        require(math.isfinite(value), f"{label}: non-finite JSON number")
    elif isinstance(value, dict):
        for key, child in value.items():
            finite_tree(child, f"{label}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            finite_tree(child, f"{label}[{index}]")


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f"duplicate JSON key: {key}")
        result[key] = value
    return result


def read_json(path):
    try:
        value = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=unique_object)
        finite_tree(value)
        return value
    except (OSError, ValueError, RecursionError) as error:
        raise InvalidReport(f"{path}: {error}") from error


def variant_config(fixture, name):
    variants = [v for v in fixture["variants"] if v["name"] == name]
    require(len(variants) == 1, f"variant: unknown name {name!r}")
    variant = variants[0]
    config = copy.deepcopy(fixture)
    for key in ("lift_end", "fold_end", "lower_end", "release_step", "retract_end"):
        config[key] = fixture["attach_step"] + math.ceil(
            (fixture[key] - fixture["attach_step"]) / variant["speed"] / 4.0
        ) * 4
    config["end_step"] = config["retract_end"] + fixture["end_step"] - fixture["retract_end"]
    return config, variant


def timing_summary(samples):
    ordered = sorted(samples)
    return {
        "p50_ms": ordered[math.ceil(0.5 * len(ordered)) - 1],
        "p95_ms": ordered[math.ceil(0.95 * len(ordered)) - 1],
        "p99_ms": ordered[math.ceil(0.99 * len(ordered)) - 1],
        "max_ms": ordered[-1], "frames": len(ordered),
        "frames_over_budget": sum(x > 1000.0 / 60.0 for x in ordered),
    }


def check_timings(run, config):
    samples = run["physics_samples_ms"]
    require(type(samples) is list and len(samples) == config["end_step"] // 4,
            "physics_samples_ms: one sample per complete four-substep frame is required")
    for sample in samples:
        require(number(sample, "physics_samples_ms") > 0, "physics sample must be positive")
    same(run["physics_frame"], timing_summary(samples), "physics_frame")
    expected = {}
    start = 0
    for phase, key in PHASES:
        end = config[key] // 4
        expected[phase] = timing_summary(samples[start:end])
        start = end
    same(run["physics_phases"], expected, "physics_phases")


def upper(value, maximum, label):
    require(number(value, label) <= maximum, f"{label}: exceeds {maximum}")


def check_surface(audit, acceptance, label):
    same(audit["crossing_pairs"], 0, f"{label}.crossing_pairs")
    upper(audit["max_separation_deficit"], acceptance["max_separation_deficit"], f"{label}.max_separation_deficit")
    return count(audit["tested_pairs"], f"{label}.tested_pairs")


def check_fold(metrics, config):
    acceptance = config["acceptance"]
    halves = metrics["projected_half_areas"]
    require(type(halves) is list and len(halves) == 2, "fold.projected_half_areas: expected two areas")
    for area in halves:
        require(number(area, "projected_half_areas") > 0, "fold: collapsed half")
    overlap = number(metrics["projected_overlap"], "fold.projected_overlap")
    require(overlap <= min(halves) + 1e-12, "fold: overlap exceeds half area")
    union = number(metrics["projected_union_area"], "fold.projected_union_area")
    # These comparisons permit only double-precision serialization/arithmetic
    # noise; acceptance limits themselves are applied without an added tolerance.
    same(union, float(sum(halves) - overlap), "fold.projected_union_area", absolute=1e-12)
    same(metrics["overlap_ratio"], overlap / min(halves), "fold.overlap_ratio", absolute=1e-12)
    same(metrics["relative_area_error"], abs(union / (config["size"] ** 2 * 0.5) - 1.0),
         "fold.relative_area_error", absolute=1e-12)
    require(acceptance["min_overlap"] <= number(metrics["overlap_ratio"], "fold.overlap_ratio") <= 1.0 + 1e-12,
            "fold.overlap_ratio: below required overlap")
    upper(metrics["relative_area_error"], acceptance["relative_area_error"], "fold.relative_area_error")
    upper(metrics["max_corner_error"], acceptance["max_corner_error"], "fold.max_corner_error")


def check_run(run, fixture):
    config, variant = variant_config(fixture, run["variant"]["name"])
    same(run["variant"], variant, "variant")
    same(run["config"], config, "config")
    same(run["failure"], None, "failure")
    same(run["failure_next_step"], None, "failure_next_step")
    same(run["steps"], config["end_step"], "steps")
    same(run["verification_enabled"], True, "verification_enabled")
    same(run["finite"], True, "finite")
    same(run["diagnostics_finite"], True, "diagnostics_finite")
    same(run["physics_settings"]["h"], config["h"], "physics_settings.h", relative=2e-7)
    same(run["physics_settings"]["iterations"], config["iterations"], "physics_settings.iterations")
    same(run["physics_settings"]["friction_model"], "persistent_material_coordinate", "physics_settings.friction_model")
    same(run["mesh"]["vertices"], 1024, "mesh.vertices")
    same(run["mesh"]["triangles"], 1922, "mesh.triangles")
    same(run["mesh"]["total_mass"], 0.05, "mesh.total_mass", relative=2e-5)
    settings = run["collision_settings"]
    for key in ("self_collision", "continuous_self_collision", "rigid_surface_collision", "continuous_rigid_collision"):
        same(settings[key], True, f"collision_settings.{key}")
    expected_values = {key: config[key] for key in ("thickness", "activation_margin", "static_friction", "kinetic_friction")}
    expected_values.update(ccd_minimum_separation=config["thickness"] * 0.9,
                           rigid_separation=config["thickness"] * 0.5,
                           rigid_ccd_minimum_separation=config["thickness"] * 0.45)
    for key, expected in expected_values.items():
        same(settings[key], expected, f"collision_settings.{key}", relative=2e-7)
    same(settings["limits"], LIMITS, "collision_settings.limits")
    work = run["collision_work_max_per_substep"]
    for key, limit in LIMITS.items():
        upper(count(work[key], f"collision_work.{key}"), limit, f"collision_work.{key}")
    count(work["limited_advances"], "collision_work.limited_advances")
    count(run["max_contacts"], "max_contacts")
    require(count(run["max_scratch_array_bytes"], "max_scratch_array_bytes") > 0, "missing scratch measurement")
    acceptance = fixture["acceptance"]
    for key, limit in (("max_strain", "max_strain"), ("max_p95_strain", "p95_strain"),
                       ("max_target_error", "max_target_error"), ("final_table_penetration", "max_table_penetration"),
                       ("settle_drift", "max_settle_drift")):
        upper(run[key], acceptance[limit], key)
    require(run["max_p95_strain"] <= run["max_strain"], "strain percentile exceeds maximum")
    number(run["max_particle_penetration"], "max_particle_penetration")
    same(run["audited_substeps"], config["end_step"], "audited_substeps")
    check_surface(run["initial_surface_audit"], acceptance, "initial_surface_audit")
    tested = check_surface(run["all_substeps_audit"], acceptance, "all_substeps_audit")
    final_tested = check_surface(run["final_surface_audit"], acceptance, "final_surface_audit")
    require(tested >= final_tested > 0, "surface audit: missing tested pairs")
    require(run["all_substeps_audit"]["max_separation_deficit"] >= run["final_surface_audit"]["max_separation_deficit"],
            "surface audit maximum excludes final state")
    task = run["task_audit"]
    same(task["observed_substeps"], config["end_step"], "task_audit.observed_substeps")
    same(task["all_substeps_finite"], True, "task_audit.all_substeps_finite")
    upper(task["max_table_penetration"], acceptance["max_table_penetration"], "task_audit.max_table_penetration")
    require(task["max_table_penetration"] >= run["final_table_penetration"], "table penetration maximum excludes final state")
    same(task["grasped_substeps"], config["release_step"] - config["attach_step"], "task_audit.grasped_substeps")
    released = config["end_step"] - config["release_step"] + 1
    same(task["released_substeps"], released, "task_audit.released_substeps")
    same(task["released_substeps_without_targets"], released, "task_audit.released_substeps_without_targets")
    same(task["settle_start_step"], config["retract_end"], "task_audit.settle_start_step")
    same(task["settle_end_step"], config["end_step"], "task_audit.settle_end_step")
    same(task["settle_samples"], config["end_step"] - config["retract_end"] + 1, "task_audit.settle_samples")
    same(task["settle_drift"], float(run["settle_drift"]), "task_audit.settle_drift")
    same(run["final_targets"], {"vertex_attachments": 0, "surface_attachments": 0, "pins": 0,
                                 "target_points": 0, "commanded_grasps": 0}, "final_targets")
    same(run["remaining_attachments"], 0, "remaining_attachments")
    same(run["remaining_pins"], 0, "remaining_pins")
    check_fold(run["fold"], config)
    check_timings(run, config)
    return variant["name"]


def check_report(report, fixture):
    try:
        finite_tree(report)
        same(report["schema_version"], 2, "schema_version")
        same(report["fixture_version"], 2, "fixture_version")
        require(report["precision"] in ("f32", "f64"), "precision: expected f32 or f64")
        same(report["dirty"], False, "dirty")
        require(type(report["commit"]) is str and re.fullmatch(r"[0-9a-f]{40}", report["commit"]), "commit: expected full Git SHA")
        for field in ("cpu", "os", "rust"):
            require(type(report[field]) is str and report[field].strip() not in ("", "unknown"), f"{field}: missing source/host metadata")
        same(report["substeps_per_frame"], 4, "substeps_per_frame")
        same(report["warmup_substeps"], 120, "warmup_substeps")
        same(report["scope"], SCOPE, "scope")
        require(type(report["runs"]) is list and len(report["runs"]) > 0, "runs: expected nonempty list")
        coverage = set()
        for index, run in enumerate(report["runs"]):
            try:
                coverage.add((report["precision"], check_run(run, fixture)))
            except (InvalidReport, KeyError, TypeError, IndexError, OverflowError) as error:
                raise InvalidReport(f"runs[{index}]: {error}") from error
        return report["commit"], coverage
    except (KeyError, TypeError, IndexError, OverflowError) as error:
        raise InvalidReport(f"missing or invalid report field: {error}") from error


def check_reports(paths, suite=False):
    fixture = read_json(FIXTURE)
    require(len({path.resolve() for path in paths}) == len(paths), "duplicate report path")
    coverage, commits = set(), set()
    for path in paths:
        try:
            commit, keys = check_report(read_json(path), fixture)
            commits.add(commit)
            coverage.update(keys)
        except InvalidReport as error:
            raise InvalidReport(f"{path}: {error}") from error
    require(len(commits) == 1, "reports must use one clean source revision")
    if suite:
        expected = {(precision, v["name"]) for precision in ("f32", "f64") for v in fixture["variants"]}
        require(coverage == expected, f"incomplete correctness suite; missing {sorted(expected - coverage)}")
    return coverage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", action="store_true", help="require all six variants in f32 and f64 from one clean revision")
    parser.add_argument("reports", type=Path, nargs="+", help="schema-2 folding JSON reports with --verify")
    args = parser.parse_args()
    try:
        coverage = check_reports(args.reports, args.suite)
    except InvalidReport as error:
        parser.exit(1, f"Folding report rejected: {error}\n")
    print(f"Recorded correctness gates passed for {len(coverage)} precision/variant combinations.")
    print("This does not qualify CPU performance, live pacing or full application memory.")


if __name__ == "__main__":
    main()
