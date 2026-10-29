#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-28/rotation_stress_v1
test ! -e "$base/surface_build_v1.log"
colcon build --symlink-install --packages-select arena_autonomy > "$base/surface_build_v1.log" 2>&1
source install/local_setup.bash
/usr/bin/python3 -B - <<'PY' > "$base/surface_installed_import_v1.log" 2>&1
import inspect
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from arena_autonomy.surface_reobservation import supported_surface
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial
assert RecoveryPolicy(True, True, True).anchor_probe.extended_surface
assert not SurfaceAnchorTrial().extended_surface
print(inspect.getsourcefile(supported_surface))
print('installed option connection verified')
PY
