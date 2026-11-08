#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B -m pytest tests/test_oblique_sweep_radius.py tests/test_oblique_gap.py tests/test_partial_observation.py tests/test_partial_lateral_metric.py tests/test_lidar_safety.py tests/test_g1_guard_partial_coupling.py tests/test_known_structure_policy.py tests/test_g1_partial_policy.py tests/test_partial_surface_alignment.py tests/test_source_provenance_r0_plus_b3.py -q > "$A/regression.log" 2>&1
r=$?;printf '%s\n' "$r" > "$A/regression.exit";exit "$r"
