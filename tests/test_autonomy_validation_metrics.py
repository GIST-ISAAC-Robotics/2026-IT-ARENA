"""깊이·추종·기록 지표가 센서 형상과 전송 성능을 혼동하지 않는지 검사합니다."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from validate_basic_autonomy import add_depth_metrics, add_rate_metrics


@pytest.mark.parametrize("rate,acceptable", [(26.7, True), (30., True), (20., False)])
def test_depth_rate_is_separate_from_geometry_and_json_safe(rate, acceptable):
    image = SimpleNamespace(encoding="32FC1", width=848, height=480,
                            header=SimpleNamespace(frame_id="camera_depth_optical_frame"))
    info = SimpleNamespace(width=848, height=480, header=image.header,
                           k=np.asarray([446., 0., 424., 0., 432., 240., 0., 0., 1.]))
    report = {}
    add_depth_metrics(report, [i/rate for i in range(101)], image, info)
    assert report["depth_interface_geometry_passed"] is True
    assert report["depth_delivery_rate_acceptable"] is acceptable
    assert report["depth_nominal_interface_passed"] is acceptable
    json.dumps(report, allow_nan=False)


def test_stereo_tracking_metrics_do_not_require_a_lidar_scan():
    trace = [{"dynamics": {"truth_longitudinal_mps": .4},
              "autonomy": {"started": True, "steering_command_rad": .1},
              "centerline_error_m": .05}]
    report = {}
    add_rate_metrics(report, trace, [], None, 10.)
    assert "lidar" not in report
    assert report["peak_ground_speed_mps"] == .4
    assert report["tracking_metrics"]["centerline_rmse_m"] == pytest.approx(.05)
    json.dumps(report, allow_nan=False)


def test_missing_depth_cannot_pass_interface_check():
    report = {}
    add_depth_metrics(report, [], None, None)
    assert report["depth_nominal_interface_passed"] is False
