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
