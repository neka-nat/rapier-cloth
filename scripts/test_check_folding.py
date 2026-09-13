"""Synthetic acceptance/rejection controls for the report checker, not physics evidence."""
import copy
from fractions import Fraction
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import tempfile
import unittest


DIRECTORY = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("check_folding", DIRECTORY / "check-folding.py")
CHECK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK)
FIXTURE = json.loads(CHECK.FIXTURE.read_text())


def synthetic_report(precision="f32", variant_index=0):
    config = copy.deepcopy(FIXTURE)
    variant = config["variants"][variant_index]
    # Independent rational schedule calculation, including slow/fast rounding.
    for key in ("lift_end", "fold_end", "lower_end", "release_step", "retract_end"):
        frames = Fraction(config[key] - config["attach_step"], 4) / Fraction(str(variant["speed"]))
        config[key] = config["attach_step"] + math.ceil(frames) * 4
    config["end_step"] = config["retract_end"] + 1200

    def constant_timing(count):
        return {"p50_ms": 1.0, "p95_ms": 1.0, "p99_ms": 1.0, "max_ms": 1.0,
                "frames": count, "frames_over_budget": 0}

    phases, start = {}, 0
    for phase, key in (("Settle", "settle_end"), ("Approach", "attach_step"), ("Lift", "lift_end"),
                       ("Fold", "fold_end"), ("Lower", "lower_end"), ("Hold", "release_step"),
                       ("Retract", "retract_end"), ("Released", "end_step")):
        phases[phase] = constant_timing((config[key] - start) // 4)
        start = config[key]
    surface = {"crossing_pairs": 0, "max_separation_deficit": 0.0, "tested_pairs": 1}
    steps, released = config["end_step"], config["end_step"] - config["release_step"] + 1
    settings = {key: config[key] for key in ("thickness", "activation_margin", "static_friction", "kinetic_friction")}
    settings.update(self_collision=True, continuous_self_collision=True, rigid_surface_collision=True,
                    continuous_rigid_collision=True, ccd_minimum_separation=0.0009,
                    rigid_separation=0.0005, rigid_ccd_minimum_separation=0.00045,
                    limits={"candidate_pairs": 2_000_000, "retained_contacts": 65_536, "ccd_checks": 2_000_000})
    run = {
        "config": config, "variant": variant, "steps": steps, "failure": None, "failure_next_step": None,
        "verification_enabled": True, "finite": True, "diagnostics_finite": True,
        "physics_settings": {"h": config["h"], "iterations": 8, "friction_model": "persistent_material_coordinate"},
        "mesh": {"vertices": 1024, "triangles": 1922, "total_mass": 0.05}, "collision_settings": settings,
        "collision_work_max_per_substep": {"candidate_pairs": 1, "retained_contacts": 1, "ccd_checks": 1, "limited_advances": 0},
        "physics_frame": constant_timing(steps // 4), "physics_samples_ms": [1.0] * (steps // 4), "physics_phases": phases,
        "max_strain": 0.0, "max_p95_strain": 0.0, "max_target_error": 0.0,
        "max_particle_penetration": 0.0, "final_table_penetration": 0.0, "max_contacts": 1, "max_scratch_array_bytes": 64,
        "fold": {"projected_half_areas": [0.125, 0.125], "projected_overlap": 0.125, "overlap_ratio": 1.0,
                 "projected_union_area": 0.125, "relative_area_error": 0.0, "max_corner_error": 0.0},
        "settle_drift": 0.0, "initial_surface_audit": copy.deepcopy(surface), "final_surface_audit": copy.deepcopy(surface),
        "all_substeps_audit": copy.deepcopy(surface), "audited_substeps": steps,
        "remaining_attachments": 0, "remaining_pins": 0,
        "final_targets": {"vertex_attachments": 0, "surface_attachments": 0, "pins": 0, "target_points": 0, "commanded_grasps": 0},
        "task_audit": {"observed_substeps": steps, "all_substeps_finite": True, "max_table_penetration": 0.0,
                       "grasped_substeps": config["release_step"] - config["attach_step"], "released_substeps": released,
                       "released_substeps_without_targets": released, "settle_samples": 1201,
                       "settle_start_step": config["retract_end"], "settle_end_step": steps, "settle_drift": 0.0},
    }
    return {"synthetic_test_only": True, "schema_version": 2, "fixture_version": 2, "precision": precision,
            "cpu": "synthetic CPU", "os": "synthetic OS", "rust": "synthetic compiler", "commit": "1" * 40,
            "dirty": False, "substeps_per_frame": 4, "warmup_substeps": 120, "scope": CHECK.SCOPE, "runs": [run]}


class FoldingCheckerTests(unittest.TestCase):
    def test_all_synthetic_variants_and_precisions(self):
        for precision in ("f32", "f64"):
            for index, variant in enumerate(FIXTURE["variants"]):
                with self.subTest(precision=precision, variant=variant["name"]):
                    _, keys = CHECK.check_report(synthetic_report(precision, index), FIXTURE)
                    self.assertEqual(keys, {(precision, variant["name"])})
        self.assertEqual(synthetic_report(variant_index=4)["runs"][0]["steps"], 3828)
        self.assertEqual(synthetic_report(variant_index=5)["runs"][0]["steps"], 3416)

    def test_unsafe_run_mutations_are_rejected(self):
        changes = [
            (("steps",), 2141), (("failure",), "prediction contact convergence"), (("failure_next_step",), 2142),
            (("verification_enabled",), False), (("audited_substeps",), 3599), (("diagnostics_finite",), False),
            (("all_substeps_audit",), None), (("initial_surface_audit", "crossing_pairs"), 1),
            (("all_substeps_audit", "crossing_pairs"), 1), (("all_substeps_audit", "tested_pairs"), 0),
            (("all_substeps_audit", "max_separation_deficit"), 0.000100001),
            (("config", "acceptance", "max_strain"), 0.5), (("config", "grid"), 16), (("config", "h"), 1 / 480),
            (("physics_settings", "iterations"), 16), (("physics_settings", "friction_model"), "kinetic_only"),
            (("collision_settings", "thickness"), 0.0005), (("collision_settings", "continuous_rigid_collision"), False),
            (("collision_settings", "limits", "ccd_checks"), 4_000_000),
            (("collision_work_max_per_substep", "candidate_pairs"), 2_000_001),
            (("max_strain",), 0.100001), (("max_p95_strain",), 0.050001), (("max_target_error",), 0.001001),
            (("max_strain",), False), (("max_target_error",), None), (("max_target_error",), -1),
            (("max_target_error",), float("nan")), (("max_target_error",), float("inf")),
            (("max_contacts",), 1.0), (("mesh", "total_mass"), 0.1), (("mesh", "triangles"), 100),
            (("final_targets", "surface_attachments"), 1), (("final_targets", "target_points"), 1),
            (("final_targets", "commanded_grasps"), 1), (("remaining_pins",), 1),
            (("task_audit", "all_substeps_finite"), False), (("task_audit", "max_table_penetration"), 0.00010001),
            (("task_audit", "grasped_substeps"), 0), (("task_audit", "released_substeps_without_targets"), 0),
            (("task_audit", "settle_samples"), 0), (("task_audit", "settle_start_step"), 3600),
            (("settle_drift",), 0.005001), (("fold", "max_corner_error"), 0.020001),
            (("fold", "projected_half_areas"), [0.0, 0.0]), (("fold", "overlap_ratio"), 0.899),
            (("fold", "relative_area_error"), 0.10001), (("fold", "projected_union_area"), 0.01),
            (("physics_frame", "p95_ms"), 0.5), (("physics_phases", "Released", "frames"), 299),
            (("physics_samples_ms",), [1.0] * 899),
        ]
        for path, value in changes:
            with self.subTest(path=path, value=value):
                report = synthetic_report()
                cursor = report["runs"][0]
                for key in path[:-1]:
                    cursor = cursor[key]
                cursor[path[-1]] = value
                with self.assertRaises(CHECK.InvalidReport):
                    CHECK.check_report(report, FIXTURE)

    def test_missing_completion_fields_are_rejected(self):
        for key in ("task_audit", "final_targets", "physics_samples_ms", "failure", "verification_enabled", "initial_surface_audit"):
            report = synthetic_report()
            del report["runs"][0][key]
            with self.subTest(key=key), self.assertRaises(CHECK.InvalidReport):
                CHECK.check_report(report, FIXTURE)
        report = synthetic_report()
        del report["runs"][0]["final_targets"]["surface_attachments"]
        with self.assertRaises(CHECK.InvalidReport):
            CHECK.check_report(report, FIXTURE)

    def test_source_metadata_must_be_present_clean_and_current_schema(self):
        for key, value in (("dirty", True), ("schema_version", 1), ("fixture_version", 1), ("commit", "unknown"),
                           ("precision", "f16"), ("cpu", ""), ("substeps_per_frame", 8), ("warmup_substeps", 0), ("runs", [])):
            report = synthetic_report()
            report[key] = value
            with self.subTest(key=key), self.assertRaises(CHECK.InvalidReport):
                CHECK.check_report(report, FIXTURE)

    def test_slow_correctness_samples_are_not_treated_as_cpu_qualification(self):
        report = synthetic_report()
        run = report["runs"][0]
        run["physics_samples_ms"] = [20.0] * 900
        for stats in [run["physics_frame"], *run["physics_phases"].values()]:
            for key in ("p50_ms", "p95_ms", "p99_ms", "max_ms"):
                stats[key] = 20.0
            stats["frames_over_budget"] = stats["frames"]
        CHECK.check_report(report, FIXTURE)

    def test_nonuniform_samples_use_nearest_rank_and_keep_every_phase(self):
        report = synthetic_report()
        run = report["runs"][0]
        run["physics_samples_ms"] = [float(i + 1) for i in range(900)]
        def stats(start, size):
            return {"p50_ms": float(start + math.ceil(size / 2)), "p95_ms": float(start + math.ceil(size * 0.95)),
                    "p99_ms": float(start + math.ceil(size * 0.99)), "max_ms": float(start + size),
                    "frames": size, "frames_over_budget": size - max(0, 16 - start)}
        run["physics_frame"] = stats(0, 900)
        start = 0
        for phase, size in (("Settle", 60), ("Approach", 30), ("Lift", 180), ("Fold", 180),
                            ("Lower", 90), ("Hold", 30), ("Retract", 30), ("Released", 300)):
            run["physics_phases"][phase] = stats(start, size)
            start += size
        CHECK.check_report(report, FIXTURE)

    def test_cli_suite_and_individual_failure_from_another_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = []
            for precision in ("f32", "f64"):
                for index in range(6):
                    path = Path(directory) / f"synthetic-{precision}-{index}.json"
                    path.write_text(json.dumps(synthetic_report(precision, index)))
                    paths.append(path)
            def invoke(*args):
                return subprocess.run(["bash", str(DIRECTORY / "check-folding.sh"), *map(str, args)],
                                      cwd=directory, capture_output=True, text=True)
            result = invoke("--suite", *paths)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("12 precision/variant", result.stdout)
            self.assertIn("does not qualify CPU", result.stdout)
            result = invoke("--suite", *paths[:-1])
            self.assertEqual(result.returncode, 1)
            self.assertIn("incomplete correctness suite", result.stderr)
            result = invoke(paths[0], paths[0])
            self.assertEqual(result.returncode, 1)
            report = synthetic_report()
            report["commit"] = "2" * 40
            paths[0].write_text(json.dumps(report))
            self.assertIn("one clean source revision", invoke("--suite", *paths).stderr)
            report["runs"][0]["failure"] = "prediction contact convergence"
            paths[0].write_text(json.dumps(report))
            result = invoke(paths[0])
            self.assertEqual(result.returncode, 1)
            self.assertIn("failure", result.stderr)

    def test_duplicate_keys_and_nonstandard_numbers_are_invalid_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            for content in ('{"failure": "bad", "failure": null}', '{"x": NaN}', '{"x": Infinity}', '{"x": 1e9999}'):
                path.write_text(content)
                with self.subTest(content=content), self.assertRaises(CHECK.InvalidReport):
                    CHECK.read_json(path)


if __name__ == "__main__":
    unittest.main()
