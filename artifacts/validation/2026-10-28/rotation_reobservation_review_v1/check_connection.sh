#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/rotation_reobservation_review_v1
test ! -e "$base/connected_contracts_v1.log"
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_small_surface_alignment.py tests/test_small_alignment_provenance.py tests/test_actor_tail_provenance.py tests/test_surface_reobservation.py tests/test_surface_reobservation_connection.py tests/test_surface_anchor_probe.py tests/test_partial_surface_alignment.py tests/test_traffic_recovery_contract.py tests/test_recovery_threat_capture.py tests/test_replay_anchor_cost.py tests/test_follow_profile_bundle.py tests/test_jetson_bundle_dependencies.py tests/test_source_provenance_r0_plus_b3.py tests/test_anchor_set_provenance.py tests/test_source_contract_helpers.py tests/test_rotation_reobservation_provenance.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py > "$base/connected_contracts_v1.log" 2>&1
colcon build --symlink-install --packages-select arena_autonomy > "$base/connected_build_v1.log" 2>&1
source install/local_setup.bash
/usr/bin/python3 -B - <<'PY' > "$base/connected_import_v1.log" 2>&1
import inspect
from arena_autonomy.surface_reobservation import small_surface_translation
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
assert not SurfaceAnchorTrial().small_alignment
assert RecoveryPolicy(True,True,True).anchor_probe.small_alignment
print(inspect.getsourcefile(small_surface_translation))
print('selected connection verified')
PY
