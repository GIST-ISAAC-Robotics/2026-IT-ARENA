"""기존 8개 DOM 계약 + 새 미리보기 재연결 3개. 실패 원본과 stage를 분리 보존."""
from pathlib import Path
import json
import subprocess

HERE=Path(__file__).resolve().parent
new=['preview_late_configuration_does_not_cross_connection','preview_late_jpeg_does_not_cross_connection',
     'preview_reconnect_clears_displayed_inputs']
existing=['units_and_pulse','auth_failure_disables_controls','reconnect_resets_binding_and_cap',
    'late_old_connection_is_ignored','late_old_snapshot_does_not_restore_wait_green',
    'old_failure_does_not_break_new_connection','stop_remains_available_without_snapshot',
    'old_command_receipt_does_not_overwrite_new_connection']
results=[]
for version,checks in [('before',new),('stage',existing+new)]:
    for name in checks:
        completed=subprocess.run(['node',str(HERE/'page_checks.js'),str(HERE/f'race_ops_page.{version}.py'),name],
            capture_output=True,text=True,encoding='utf-8',timeout=10)
        results.append(dict(version=version,check=name,passed=completed.returncode==0,
            stdout=completed.stdout,stderr=completed.stderr))
(HERE/'dom_results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
summary={v:dict(total=sum(r['version']==v for r in results),passed=sum(r['version']==v and r['passed'] for r in results))
    for v in ['before','stage']}
print(json.dumps(summary,indent=2))
assert summary['before']['passed']==0 and summary['stage']['passed']==11
