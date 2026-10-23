"""Installed-file/import audit only: no node construction or controller execution."""
import hashlib
import importlib
import importlib.metadata
import json
from pathlib import Path
import sys

import rclpy
import rclpy.node
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def forbidden(*args, **kwargs):
    raise AssertionError("ROS node construction is forbidden during install audit")


rclpy.init = forbidden
rclpy.create_node = forbidden
rclpy.node.Node.__init__ = forbidden
root = Path(sys.argv[1]).resolve()
output = Path(sys.argv[2])
expected_prefix = Path(sys.argv[3]).resolve() if len(sys.argv) > 3 else None
checks = []
packages = {}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


for name in ("arena_autonomy", "arena_vehicle_interface"):
    package = importlib.import_module(name)
    location = Path(package.__file__).resolve().parent
    packages[name] = {"module_path": str(location), "prefix": get_package_prefix(name)}
    if expected_prefix is not None:
        checks.append({"name": name + ":isolated_import", "ok": location.is_relative_to(expected_prefix)})
    for source in sorted((root / "src" / name / name).rglob("*.py")):
        if "__pycache__" in source.parts:
            continue
        rel = source.relative_to(root / "src" / name / name)
        installed = location / rel
        checks.append({"name": f"{name}/{rel}", "ok": installed.is_file() and digest(source) == digest(installed)})
    distribution = importlib.metadata.distribution(name)
    for entry in distribution.entry_points:
        if entry.group == "console_scripts":
            value = entry.load()  # Load callable; never call main().
            checks.append({"name": f"entry:{entry.name}", "ok": callable(value)})

for name, subdirectories in (("arena_bringup", ("launch", "config")),
                            ("arena_description", ("config", "models")),
                            ("arena_gazebo", ("worlds",))):
    location = Path(get_package_share_directory(name)).resolve()
    packages[name] = {"share_path": str(location), "prefix": get_package_prefix(name)}
    if expected_prefix is not None:
        checks.append({"name": name + ":isolated_share", "ok": location.is_relative_to(expected_prefix)})
    for directory in subdirectories:
        for source in sorted((root / "src" / name / directory).rglob("*")):
            if not source.is_file() or "__pycache__" in source.parts:
                continue
            rel = source.relative_to(root / "src" / name)
            installed = location / rel
            checks.append({"name": f"{name}/{rel}", "ok": installed.is_file() and digest(source) == digest(installed)})

result = {"packages": packages, "sys_path": sys.path, "checks": checks,
          "passed": all(item["ok"] for item in checks),
          "nodes_started": False, "runtime_or_hardware_verified": False}
with output.open("x", encoding="utf-8") as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2)
print(json.dumps({"passed": result["passed"], "checks": len(checks),
                  "failed": [item for item in checks if not item["ok"]]}))
raise SystemExit(0 if result["passed"] else 1)
