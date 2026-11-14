#!/usr/bin/env bash
# Extracted package only; no ROS graph, serial, OpenOCD or hardware access.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_vehicle_interface:$PWD/src/arena_autonomy${PYTHONPATH:+:$PYTHONPATH}"
result_dir="$1"
python3 -B scripts/deploy_uno_q_bundle.py . > "$result_dir/package_inventory.json"
python3 -B scripts/uno_q_host.py --dry-run > "$result_dir/package_host_dryrun.json"
python3 -B scripts/uno_q_host.py --dry-run --bench --bench-duty-permille 50 --bench-direction 0 --bench-duration-ms 5000 > "$result_dir/package_bench_dryrun.json"
python3 -B firmware/bench/flash_native.py --firmware-dir firmware/bench --backup-dir "$result_dir/no_backup_should_be_created" > "$result_dir/package_flash_dryrun.json"
test ! -e "$result_dir/no_backup_should_be_created"
python3 -B scripts/prepare_uno_q_quick_setup.py --output "$result_dir/package_quick_setup.yaml" > "$result_dir/package_quick_setup.json"
cmp config/hardware/uno_q_quick_setup_2026_11_14.yaml "$result_dir/package_quick_setup.yaml"
python3 -B -m pytest tests/test_uno_q_bundle.py tests/test_uno_q_quick_setup.py tests/test_uno_q_host*.py tests/test_imu_quick_calibration*.py tests/test_quick_self_mask.py -q -p no:cacheprovider --basetemp=/tmp/it-arena-uno-q-package-20261114 --junitxml="$result_dir/package_pytest.xml"
python3 -B scripts/deploy_uno_q_bundle.py . > "$result_dir/package_inventory_after.json"
