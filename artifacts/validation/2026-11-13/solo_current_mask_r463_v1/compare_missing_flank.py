"""일반 결측 반례는 새 구조 허용뿐 아니라 원래의 저속 정책과 비교한다."""
import copy,json,math,sys
from pathlib import Path
A=Path(__file__).resolve().parent
which=sys.argv[1]
assert which in ('base','stage')
sys.path[:0]=[str(A/which/'src/arena_autonomy'),str(A/which/'src/arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.runtime_dual_state import current_request
from arena_autonomy.follow_trace import freeze
d=json.loads((A/'inputs.json').read_bytes());r=d['calls'][-1]['data']
current=copy.deepcopy(r['current_rebase']['input'])
current['scan']['measurements']['ranges_m'][424]=freeze(math.inf)
request=current_request(r['output'],r['compute']['input'],current,
    retained=r['compute']['route_check']['retained'],crop_current_selection=True)
before=json.dumps(request,sort_keys=True)
e=Evaluator('current',d['header'])
try: answer=e(request)
finally:e.close()
assert before==json.dumps(request,sort_keys=True)
(A/f'missing_flank_{which}.json').write_text(json.dumps(answer,indent=2),encoding='utf-8')
print(json.dumps(dict(source=which,speed=answer['speed_mps'],reason=answer['reason'])))
