"""전용 사본에서 한 번씩 실행하고 exit/원본 불변을 남긴다."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parent
manifest=json.loads((ROOT/'manifest.json').read_bytes())
def changed():return [name for name,want in manifest.items() if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=want]
assert not changed()
out=ROOT/'unit_result';out.mkdir(exist_ok=False)
reference='legacy/tests/reference_r423_scope.py'
selection=[reference+'::'+name for name in (
    'test_same_scan_compact_contract_and_no_input_mutation',
    'test_actual_parent_offer_uses_owned_pair_and_leaves_inputs_unchanged',
    'test_original_timing_is_preserved_and_rejected',
    'test_false_or_other_plan_markers_do_not_select[false_allowed]',
    'test_false_or_other_plan_markers_do_not_select[latch]',
    'test_whole_scan_metadata_difference_recomputed_at_parent[acquired_at_s]')]
steps=[]
for label,command,timeout in (
    ('units',[sys.executable,'-B','-m','pytest','-q','-p','no:cacheprovider','legacy/tests/test_crop_current_reach.py',*selection,
        'tests/test_crawl_evaluator.py','--junitxml=unit_result/junit.xml'],120),
    ('saved',[sys.executable,'-B','verify_saved.py'],180)):
    started=time.monotonic()
    with (ROOT/(label+'.log')).open('xb') as stream:
        child=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
        forced=False
        try:code=child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:forced=True;child.kill();code=child.wait()
    steps.append(dict(label=label,pid=child.pid,command=command,exitcode=code,forced=forced,reaped=True,elapsed_s=time.monotonic()-started))
    if code:break
report=dict(passed=len(steps)==2 and all(s['exitcode']==0 and not s['forced'] for s in steps) and not changed(),
    steps=steps,source_changes=changed(),ros_started=False,devices_opened=False)
(ROOT/'execution.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report))
raise SystemExit(0 if report['passed'] else 1)
