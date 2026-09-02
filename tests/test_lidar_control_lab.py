"""LiDAR 고정 속도 시험 평가가 미도달·이탈·표본 단절을 통과시키지 않는지 확인합니다."""
import importlib.util
import ast
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("lidar_lab", REPO / "scripts/validate_lidar_control_lab.py")
lab = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lab)


def state(x=0., y=0., yaw=0.):
    return {"truth_x_m": x, "truth_y_m": y, "truth_yaw_rad": yaw,
            "truth_longitudinal_mps": 0., "truth_lateral_mps": 0.}


class LidarLabTests(unittest.TestCase):
    def test_controller_has_no_truth_or_route_dependency(self):
        source = (REPO / "scripts/validate_lidar_control_lab.py").read_text(encoding="utf-8")
        controller = next(node for node in ast.walk(ast.parse(source))
                          if isinstance(node, ast.ClassDef) and node.name == "LidarOnlyController")
        body = ast.get_source_segment(source, controller)
        for forbidden in ("truth_", "/sim/drivetrain", "centerline.csv", "observer", "latest", "radius_m"):
            self.assertNotIn(forbidden, body)

    def test_straight_footprint_and_heading(self):
        result = lab.footprint_metrics(state(y=.06), None)
        self.assertAlmostEqual(result["centerline_error_m"], .06)
        self.assertAlmostEqual(result["road_clearance_m"], .08)
        perpendicular = lab.footprint_metrics(state(y=.13, yaw=math.pi / 2), None)
        self.assertLess(perpendicular["road_clearance_m"], 0)
        self.assertGreater(perpendicular["wall_clearance_m"], 0)

    def test_circle_inner_edge_not_only_corners(self):
        result = lab.footprint_metrics(state(x=1., yaw=math.pi / 2), 1.)
        self.assertAlmostEqual(result["centerline_error_m"], 0.)
        expected = 1.225 - math.hypot(1.085, .1)
        self.assertAlmostEqual(result["road_clearance_m"], expected)
        inside = lab.footprint_metrics(state(x=.80, yaw=math.pi / 2), 1.)
        self.assertAlmostEqual(inside["road_clearance_m"], .80 - .085 - .775)

    def test_duration_breaks_on_sample_gaps(self):
        rows = [{"sim_time_s": value} for value in [0., .05, .10, 1., 1.05]]
        self.assertAlmostEqual(lab.longest_duration(rows, lambda _: True), .1)
        self.assertEqual(lab.longest_duration(rows, lambda _: False), 0.)

    def test_lab_world_keeps_original_and_uses_collisions(self):
        source = REPO / "src/arena_gazebo/worlds/vehicle_dynamics_lab/world.sdf"
        before = source.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            for case, count in (("straight_20kmh", 2), ("circle_5kmh", 720)):
                path = lab.make_world(Path(directory), lab.CASES[case])
                root = ET.parse(path)
                self.assertEqual(len(root.findall(".//model[@name='lidar_lab_walls']/link/collision")), count)
                self.assertEqual(root.findtext("world/physics/max_step_size"), "0.001")
        self.assertEqual(source.read_bytes(), before)

    def test_empty_trace_is_not_pass(self):
        with self.assertRaises(RuntimeError):
            lab.summarize([], [], [], lab.CASES["straight_20kmh"], 10.)

    def test_speed_peak_without_sustained_dwell_is_not_pass(self):
        case = lab.CASES["straight_20kmh"]
        trace = []
        for index in range(460):
            now = index * .02
            speed = case["speed_mps"] if index == 100 else 1. if now < 8. else 0.
            trace.append({"sim_time_s": now, "elapsed_s": now,
                "phase": "running" if now < 8. else "stopping",
                "truth_longitudinal_mps": speed, "truth_lateral_mps": 0.,
                "planar_speed_mps": speed, "wheel_surface_speed_mps": speed,
                "truth_roll_rad": 0., "truth_pitch_rad": 0.,
                "centerline_error_m": 0., "road_clearance_m": .1, "wall_clearance_m": .3})
        controls = [{"phase": "running", "time_s": index * .01, "scan_age_s": .03,
            "scan_stamp_s": (index // 10) * .1, "steering_rad": 0., "reason": "following"} for index in range(800)]
        scans = [{"stamp_s": index * .1, "scan_time_s": 0., "time_increment_s": 0.,
                  "samples": 500, "frame_id": "laser_frame"} for index in range(92)]
        result = lab.summarize(trace, controls, scans, case, 10.)
        self.assertAlmostEqual(result["peak_longitudinal_speed_kmh"], 20.)
        self.assertFalse(result["target_speed_verified"])
        self.assertFalse(result["fixed_speed_tracking_passed"])
        for row in trace:
            if row["phase"] == "running":
                row["truth_longitudinal_mps"] = row["planar_speed_mps"] = row["wheel_surface_speed_mps"] = case["speed_mps"]
        result = lab.summarize(trace, controls, scans, case, 10.)
        self.assertTrue(result["fixed_speed_tracking_passed"])
        trace[-1]["road_clearance_m"] = -.01
        result = lab.summarize(trace, controls, scans, case, 10.)
        self.assertFalse(result["fixed_speed_tracking_passed"])
        self.assertEqual(result["road_departure_samples_active"], 0)
        self.assertEqual(result["road_departure_samples_including_stop"], 1)


if __name__ == "__main__":
    unittest.main()
