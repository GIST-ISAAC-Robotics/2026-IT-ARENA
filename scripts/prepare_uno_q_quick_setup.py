#!/usr/bin/env python3
"""기존 ROS 설정을 보존하며 UNO Q 자동 보정·자기 영역 제외 overlay를 만든다."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def create_parameters(calibration, mask, base=None):
    if (not isinstance(calibration, dict) or set(calibration) != {"/**"}
            or not isinstance(calibration["/**"], dict)
            or not isinstance(calibration["/**"].get("ros__parameters"), dict)):
        raise ValueError("calibration must be a global ROS parameter overlay")
    mask_keys = {"schema", "enabled", "frame_id", "status", "center_xy_m", "body_length_m", "body_width_m", "margin_each_side_m"}
    if (not isinstance(mask, dict) or set(mask) != mask_keys or mask.get("schema") != "arena_quick_self_mask_v1"
            or mask.get("enabled") is not True or mask.get("frame_id") != "base_link"
            or mask.get("status") != "temporary_user_approximation_unmeasured"
            or mask.get("body_length_m") != .20 or mask.get("body_width_m") != .15
            or mask.get("margin_each_side_m") != .03 or mask.get("center_xy_m") != [.00365, 0.]):
        raise ValueError("confirmed 20x15 cm plus 3 cm each-side profile required")
    overrides = copy.deepcopy(calibration["/**"]["ros__parameters"])
    overrides.update(lidar_self_mask_json=json.dumps(mask, ensure_ascii=False, separators=(",", ":")),
                     motion_imu_topic="/hardware_runtime/imu", motion_imu_frame_id="lsm6dsox_sensor",
                     rear_blind_half_angle_deg=0., lidar_blind_sectors_json="",
                     known_structure_policy="off", g1_g2_rear_wall_witness=False)
    result = {} if base is None else copy.deepcopy(base)
    if not isinstance(result, dict):
        raise ValueError("base parameters must be a ROS mapping")
    result.setdefault("/**", {"ros__parameters": {}})
    for name, value in result.items():
        if not isinstance(name, str) or not isinstance(value, dict) or not isinstance(value.get("ros__parameters"), dict):
            raise ValueError("only ROS node/global parameter sections are accepted")
        # Explicit node sections must not shadow the selected quick settings.
        value["ros__parameters"].update(copy.deepcopy(overrides))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-params", type=Path, help="선택: 보존할 현재 ROS parameter YAML")
    parser.add_argument("--calibration", type=Path, default=ROOT / "config/hardware/imu_quick_calibration.yaml")
    parser.add_argument("--mask", type=Path, default=ROOT / "config/sensors/quick_self_mask_2026_11_14.json")
    parser.add_argument("--output", type=Path, required=True, help="새 ROS parameter YAML")
    args = parser.parse_args(argv)
    calibration = yaml.safe_load(args.calibration.read_text(encoding="utf-8"))
    mask = json.loads(args.mask.read_text(encoding="utf-8"))
    base = None if args.base_params is None else yaml.safe_load(args.base_params.read_text(encoding="utf-8"))
    result = create_parameters(calibration, mask, base)
    text = yaml.safe_dump(result, allow_unicode=True, sort_keys=False)
    with args.output.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(text)
    print(json.dumps({"output": str(args.output), "sha256": hashlib.sha256(text.encode()).hexdigest(),
                      "base_preserved_on_disk": True, "mask_extent_m": [.26, .21],
                      "mask_center_xy_m": [.00365, 0.], "hardware_access": False,
                      "auto_arm": False, "motor_profile_changed": False}, ensure_ascii=False))


if __name__ == "__main__":
    main()
