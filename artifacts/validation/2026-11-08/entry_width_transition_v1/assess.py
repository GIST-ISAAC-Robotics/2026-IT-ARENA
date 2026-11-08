"""평가 전용 r40. r33 assess.py 판정을 그대로 실행한 뒤 후보 설정·정렬 불변식만 별도로 덧붙인다."""
import json,math
from r33_tools import HERE,execute

try:execute('assess.py')
except SystemExit as done:base_exit=done.code
out=json.loads((HERE/'assessment.json').read_text(encoding='utf-8'))

def found(value,key):
    if isinstance(value,dict):
        return ([value[key]] if key in value else [])+[x for v in value.values() for x in found(v,key)]
    if isinstance(value,list):return [x for v in value for x in found(v,key)]
    return []

applied=alignment=selected=invalid=0;lookahead={}
with (HERE/'run1/follow_decisions.jsonl').open(encoding='utf8') as stream:
    header=json.loads(next(stream))['data']
    for line in stream:
        event=json.loads(line)
        if event['kind']!='call':continue
        inp,result=event['data']['input'],event['data']['output']
        previews=[p for p in found(result[2],'adaptive_preview') if isinstance(p,dict) and p.get('applied')]
        applied+=bool(previews)
        for p in previews:lookahead[str(p.get('lookahead_m'))]=lookahead.get(str(p.get('lookahead_m')),0)+1
        selected+=any(not p.get('alignment_only') for p in previews)
        if found(result[2],'preview_alignment'):
            alignment+=1
            # 정렬은 속도 0 명령이며 정지 입력·기존 3 rad/s 변화율 안에서만 허용한다.
            if (result[0]!=0 or inp['speed_mps']!=0
                    or abs(result[1]-inp['previous_steering_rad'])>3*inp['dt_s']+1e-8 or not math.isfinite(result[1])):
                invalid+=1
candidate=dict(adaptive_preview_enabled=header['execution_config'].get('adaptive_entry_preview') is True,
               preview_alignment_zero_speed_and_rate=invalid==0)
out.update(r33_gates_passed=out['passed'],r33_exit=base_exit,candidate_gates=candidate,
    passed=out['passed'] and all(candidate.values()),
    preview=dict(applied_calls=applied,positive_short_preview_calls=selected,alignment_calls=alignment,
                 alignment_invalid=invalid,lookahead_counts=lookahead,
                 note='alignment calls are speed-0 steering requests, not bypass success'),
    scope=out['scope']+'; r40 = r33 + adaptive_entry_preview, built on r39 with bounded current corridor offset comparison')
(HERE/'assessment.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(passed=out['passed'],r33_gates=out['gates'],candidate=candidate,preview=out['preview']),ensure_ascii=True))
raise SystemExit(0 if out['passed'] else 1)
