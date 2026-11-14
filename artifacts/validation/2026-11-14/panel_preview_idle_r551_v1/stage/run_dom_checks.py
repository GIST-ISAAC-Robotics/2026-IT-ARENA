from pathlib import Path
import json
import subprocess

HERE=Path(__file__).resolve().parent
new=['hidden_tab_keeps_ops_poll_without_preview','unselected_camera_does_not_fetch_and_discard',
     'jpeg_finishing_while_hidden_is_not_displayed']
old=['units_and_pulse','auth_failure_disables_controls','reconnect_resets_binding_and_cap',
     'late_old_connection_is_ignored','late_old_snapshot_does_not_restore_wait_green',
     'old_failure_does_not_break_new_connection','stop_remains_available_without_snapshot',
     'old_command_receipt_does_not_overwrite_new_connection',
     'preview_late_configuration_does_not_cross_connection','preview_late_jpeg_does_not_cross_connection',
     'preview_reconnect_clears_displayed_inputs']
results=[]
for name,cases in [('race_ops_page.py.before',new),('race_ops_page.py',old+new)]:
 for case in cases:
  result=subprocess.run(['node',str(HERE/'page_checks.js'),str(HERE/name),case],capture_output=True,text=True,encoding='utf-8',timeout=10)
  results.append(dict(source=name,check=case,passed=result.returncode==0,stdout=result.stdout,stderr=result.stderr))
(HERE/'dom_results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
summary={name:dict(total=sum(r['source']==name for r in results),passed=sum(r['source']==name and r['passed'] for r in results)) for name in ['race_ops_page.py.before','race_ops_page.py']}
print(json.dumps(summary,indent=2))
assert summary['race_ops_page.py.before']['passed']==0
assert summary['race_ops_page.py']['passed']==14
