"""실행 출력의 호길이 확인과 실제 회랑 채택을 구분한다."""
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent


def usage():
    path=HERE/'run1/follow_decisions.jsonl'
    digest=sha256();counts=Counter();selected=[];footer=None;sequence=-1
    with path.open('rb') as stream:
        for line in stream:
            digest.update(line);event=json.loads(line)
            if footer is not None or event['seq']!=sequence+1:raise ValueError('trace order/footer')
            sequence=event['seq'];data=event['data']
            if event['kind']=='header':
                if not data['follow_config']['corridor_arc_agreement']:raise ValueError('option missing')
            elif event['kind']=='footer':footer=data
            elif event['kind']=='call':
                v,a,d=data['output'];c=d.get('corridor_candidate') or {};method=c.get('path_agreement_method')
                counts['calls']+=1;counts['comparison_'+str(method)]+=1
                if d.get('corridor_selected'):
                    counts['corridor_selected']+=1
                    selected.append(dict(t=data['input']['reference_s'],speed_mps=v,steering_rad=a,
                        agreement=method,reason=d.get('reason'),underlying_reason=d.get('underlying_reason')))
    if (footer is None or footer['written']!=sequence or footer['accepted']!=sequence
            or footer['worker_alive'] is not False or any(footer[k] for k in ('failed','dropped','capture_errors','pending'))):
        raise ValueError('incomplete trace')
    return dict(scope='호길이 비교 이력과 현재 회랑 출력 채택의 실행 기록. 호길이 비교 호출 전부가 출력 변경은 아니다.',
        counts=dict(counts),selected=selected,input_sha256=digest.hexdigest(),
        script_sha256=sha256(Path(__file__).read_bytes()).hexdigest())


if __name__=='__main__':
    result=usage()
    with (HERE/'usage.json').open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps(result['counts']))
