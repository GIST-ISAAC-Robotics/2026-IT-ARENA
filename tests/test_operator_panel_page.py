"""실제 패널 script의 비동기 수신/연결 상태. ROS·실차·브라우저 레이아웃 검증 아님."""
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.mark.parametrize('case', [
    'units_and_pulse', 'auth_failure_disables_controls', 'reconnect_resets_binding_and_cap',
    'late_old_connection_is_ignored', 'late_old_snapshot_does_not_restore_wait_green',
    'old_failure_does_not_break_new_connection', 'stop_remains_available_without_snapshot',
    'old_command_receipt_does_not_overwrite_new_connection',
    'preview_late_configuration_does_not_cross_connection',
    'preview_late_jpeg_does_not_cross_connection',
    'preview_reconnect_clears_displayed_inputs',
    'hidden_tab_keeps_ops_poll_without_preview',
    'unselected_camera_does_not_fetch_and_discard',
    'jpeg_finishing_while_hidden_is_not_displayed',
])
def test_operator_panel_page(case):
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js가 필요한 로컬 DOM 검사')
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run([
        node, str(root/'tests/operator_panel_page_checks.js'),
        str(root/'src/arena_autonomy/arena_autonomy/race_ops_page.py'), case,
    ], capture_output=True, text=True, encoding='utf-8', timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
