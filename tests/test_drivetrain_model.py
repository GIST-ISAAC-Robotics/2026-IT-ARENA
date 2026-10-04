import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

REPO = Path(__file__).resolve().parents[1]
XACRO = REPO / "src/arena_description/models/arena_car/model.sdf.xacro"


def model(**mappings):
    return ET.fromstring(xacro.process_file(str(XACRO), mappings=mappings).toxml()).find("model")


def test_default_is_one_motor_force_drive_and_legacy_is_explicit():
    root = model()
    assert root.find("plugin[@name='arena::SingleMotorDrive']") is not None
    assert root.find("plugin[@name='gz::sim::systems::AckermannSteering']") is None
    legacy = model(drive_mode="legacy_velocity")
    assert legacy.find("plugin[@name='arena::SingleMotorDrive']") is None
    assert legacy.find("plugin[@name='gz::sim::systems::AckermannSteering']") is not None
    source = (REPO / "src/arena_gazebo/src/single_motor_drive.cpp").read_text()
    assert ".SetForce(" in source
    assert "JointVelocityCmd" not in source and ".SetVelocity(" not in source


def test_mass_and_geometry_survive_sensor_rendering_toggle():
    for root in (model(), model(render_sensors="false")):
        assert sum(float(link.findtext("inertial/mass")) for link in root.findall("link")) == pytest.approx(2.0)
        assert root.findtext("link[@name='chassis']/collision/geometry/box/size") == "0.2 0.15 0.06"
    assert len(model(render_sensors="false").findall(".//sensor")) == 1  # IMU는 유지


def test_wheel_friction_axes_and_speed_allow_twenty_kmh_without_forcing_it():
    root = model()
    for link in root.findall("link"):
        if link.get("name").endswith("_wheel"):
            assert link.findtext("collision/surface/friction/ode/fdir1") == "0 0 1"
    for joint in root.findall("joint"):
        if joint.get("name").endswith("_wheel_joint"):
            assert float(joint.findtext("axis/limit/velocity")) > 20 / 3.6 / .025
    slip = root.find("plugin[@name='gz::sim::systems::WheelSlip']")
    assert len(slip.findall("wheel")) == 4


def test_launch_passes_selected_differential_and_rejects_bad_efficiency():
    spec = importlib.util.spec_from_file_location("dynamics_launch", REPO / "src/arena_bringup/launch/simulation.launch.py")
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    config = yaml.safe_load((REPO / "src/arena_description/config/vehicle.yaml").read_text())["vehicle"]
    for profile in launch.DIFFERENTIAL_PROFILES:
        maps = launch._dynamics_mappings(config, "single_motor", profile)
        plugin = model(**maps).find("plugin[@name='arena::SingleMotorDrive']")
        assert float(plugin.findtext("gear_efficiency")) == config["drivetrain"]["mechanical_differential"]["profiles"][profile]["gear_efficiency"]
        assert (float(plugin.findtext("differential_torque_limit")) > 0) == (profile == "viscous_lsd")
    config["drivetrain"]["mechanical_differential"]["profiles"]["ideal_open"]["gear_efficiency"] = 1.1
    with pytest.raises(ValueError):
        launch._dynamics_mappings(config)


def load_launch():
    spec = importlib.util.spec_from_file_location("feedback_launch", REPO / "src/arena_bringup/launch/simulation.launch.py")
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    return launch


def vehicle():
    return yaml.safe_load((REPO / "src/arena_description/config/vehicle.yaml").read_text(encoding="utf-8"))["vehicle"]


def test_selected_scan_plane_stays_above_chassis_with_65mm_wheels():
    config = vehicle()
    profile = yaml.safe_load((REPO / 'src/arena_description/config/b0183_c1.yaml').read_text(encoding='utf-8'))
    root = model(**load_launch()._dynamics_mappings(config))
    chassis = root.find("link[@name='chassis']")
    center_z = float(chassis.findtext('pose').split()[2])
    height = float(chassis.findtext('collision/geometry/box/size').split()[2])
    clearance = profile['lidar']['xyz_m'][2] - (center_z + height / 2)
    assert clearance == pytest.approx(.0025)
    assert clearance > 0  # 현재 상자 형상만의 검사. 마운트·공차·실물 가림 보증이 아니다.


def test_selected_motor_gearing_and_wheel_reach_generated_model():
    config = vehicle()
    launch = load_launch()
    drive, motor = config["drivetrain"], config["drivetrain"]["motor"]
    assert (drive["wheel_radius_m"], drive["wheelbase_m"], motor["gear_ratio"]) == (.0325, .135, 15.)
    assert motor["free_speed_rad_s"] == pytest.approx(18000 * 2 * 3.141592653589793 / 60)
    # 임시 토크 상한은 정격보다 낮고, 두 사양점 선형 외삽 정지 토크(약 1.25 N·m)로 올리지 않는다.
    spec = motor["specification_points_not_measured"]
    no_load, rated = spec["no_load_rpm"], spec["rated_rpm"]
    stall_extrapolation = spec["rated_torque_nm"] * no_load / (no_load - rated)
    assert stall_extrapolation == pytest.approx(1.25, abs=.01)
    assert motor["torque_limit_nm"] == .060 < spec["rated_torque_nm"] < stall_extrapolation
    root = model(**launch._dynamics_mappings(config), wheelbase=str(drive["wheelbase_m"]),
                 wheel_radius=str(drive["wheel_radius_m"]))
    plugin = root.find("plugin[@name='arena::SingleMotorDrive']")
    assert float(plugin.findtext("gear_ratio")) == 15.
    assert float(plugin.findtext("wheel_radius")) == .0325
    assert float(plugin.findtext("wheelbase")) == .135
    assert float(plugin.findtext("motor_torque_limit")) == .060
    assert float(plugin.findtext("motor_free_speed")) == pytest.approx(1884.9555921539)
    for link in root.findall("link"):
        if link.get("name").endswith("_wheel"):
            assert float(link.findtext("collision/geometry/cylinder/radius")) == .0325
    # xacro 기본값도 같은 값이어야 매핑 없는 모델 생성 검사가 옛 기하를 쓰지 않는다.
    default = model().find("plugin[@name='arena::SingleMotorDrive']")
    assert float(default.findtext("gear_ratio")) == 15. and float(default.findtext("wheelbase")) == .135
    header = (REPO / "src/arena_gazebo/include/arena_gazebo/drivetrain.hpp").read_text(encoding="utf-8")
    assert "radius{0.0325}" in header and "ratio{15.0}" in header and "freeSpeed{1884.9555921539}" in header


def test_local_c1_scan_plane_stays_above_raised_chassis_top():
    config = vehicle()
    profile = yaml.safe_load((REPO / "src/arena_description/config/b0183_c1.yaml").read_text(encoding="utf-8"))
    chassis_top = config["drivetrain"]["wheel_radius_m"] + config["body"]["height_m"]
    margin = profile["lidar"]["xyz_m"][2] - chassis_top
    # 65 mm 바퀴로 차체 상자 상단이 0.0925 m가 되어 여유가 10 mm에서 2.5 mm로 줄었다.
    assert margin == pytest.approx(.0025, abs=1e-9) and margin > 0


def test_drive_feedback_resolution_for_motor_and_explicit_legacy():
    config = vehicle()
    launch = load_launch()
    assert launch.DRIVE_FEEDBACK_MODES == ("drive_motor_shaft", "rear_wheel_pair_legacy")
    motor = launch._drive_feedback(config)
    assert motor["mode"] == "drive_motor_shaft" and motor["topic"] == "/drive_motor/encoder"
    assert motor["encoder_node"]["counts_per_motor_revolution"] == 28
    assert motor["encoder_node"]["gear_ratio"] == 15. and motor["encoder_node"]["wheel_radius_m"] == .0325
    assert "ticks_per_revolution" not in motor["encoder_node"]
    assert motor["mcu"] == {"feedback_mode": "drive_motor_shaft", "wheel_radius_m": .0325,
                            "ticks_per_revolution": 28, "gear_ratio": 15.}
    assert motor["motion"] == {"motion_feedback_mode": "drive_motor_shaft", "motion_gear_ratio": 15.,
                               "motion_wheel_radius_m": .0325}
    assert motor["resolution"]["counts_per_mean_wheel_revolution"] == 420
    assert motor["resolution"]["mean_wheel_travel_m_per_count"] * 1000 == pytest.approx(.4862, abs=1e-4)
    legacy = launch._drive_feedback(config, "rear_wheel_pair_legacy")
    assert legacy["topic"] == "/wheel_states" and legacy["mcu"]["gear_ratio"] == 1.
    assert legacy["encoder_node"]["ticks_per_revolution"] == 2048
    with pytest.raises(ValueError):
        launch._drive_feedback(config, "configured_typo")
    broken = vehicle()
    broken["sensors"]["drive_feedback"]["drive_motor_encoder"]["counts_per_motor_revolution"] = 2048
    with pytest.raises(ValueError):
        launch._drive_feedback(broken)
    broken = vehicle()
    broken["sensors"]["drive_feedback"]["drive_motor_encoder"]["dropout_probability"] = 1.5
    with pytest.raises(ValueError):
        launch._drive_feedback(broken)


def test_launch_routes_one_feedback_contract_to_every_consumer():
    source = (REPO / "src/arena_bringup/launch/simulation.launch.py").read_text(encoding="utf-8")
    assert '["wheel_encoders"]' not in source
    assert source.count('**feedback["motion"]') == 2          # lidar_safety + 자율주행 제어기
    assert "**feedback['mcu']" in source                      # virtual MCU 실제 피드백 경로
    assert '{**feedback["encoder_node"], "use_sim_time": True}' in source
    assert '"drive_feedback_mode": feedback["mode"]' in source  # ToF 보호층
    assert "'feedback_mode': feedback['mode']" in source       # 고장 주입 중계
    assert '(feedback["topic"], feedback["raw_test_topic"])' in source
    assert "DeclareLaunchArgument('drive_feedback_mode'" in source
