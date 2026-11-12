"""완료 알림 후보와 원 부모 재검사 결과를 비교하고 긴 수신 지연을 집계한다."""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[4]
A = Path(__file__).resolve().parent
B = A.parent/'two_stage_recheck_r235_v1'
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import equal_output

left = json.loads((B/'jetson_parent/answers.json').read_bytes())
right = json.loads((A/'jetson_event/answers.json').read_bytes())
assert len(left) == len(right) == 632
differences = [a[0] for a, b in zip(left, right) if not equal_output(a, b)]
report = json.loads((A/'jetson_event/report.json').read_bytes())
assert report['execution_complete'] and report['missing'] == report['unexpected'] == report['duplicates'] == 0
assert report['completion_notice']['alive'] is False and report['completion_notice']['fault'] is None
assert report['cleanup'] == {'forced': False, 'exitcode': 0}
long_delays = json.loads((A/'jetson_event/long_delays.json').read_bytes())
assert len(long_delays) == report['delays_30ms_or_more']
result = dict(full_output_equal=not differences, differences=differences, plans=len(right),
    rechecks=sum(r[2] is not None for r in right),
    long_delay_count=len(long_delays), long_delay_without_gc=sum(not r['gc_overlap'] for r in long_delays),
    gc_generation2_ms=report['gc_generation2_ms'],
    actual_ros_domain=196, raw_report_scope_text_has_old_domain197=True,
    scope='original report retained; environment guard enforces196; no real vehicle, fresh-input gate or actuation')
(A/'comparison.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
print(json.dumps(result))
assert not differences
