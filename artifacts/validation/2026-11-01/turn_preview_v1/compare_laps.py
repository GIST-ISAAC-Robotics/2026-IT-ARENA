"""같은 2 m/s 상한 두 랩의 실제 진행 위치별 통과 시간과 속도 보완 호출을 비교한다."""
import json
import math
import sys
from collections import Counter
from pathlib import Path

BASE = Path(__file__).resolve().parent
PREVIOUS = BASE.parent/'corner_approach_v1'
sys.path.insert(0, str(PREVIOUS))
from lap_budget import summarize


def lap(folder):
    report=json.loads((folder/'report.json').read_text(encoding='utf-8'))
    length=report['lap_length_m']
    targets=[length*k/8 for k in range(1,9)]
    crossings=[];prev=None;start=None
    with (folder/'trajectory.jsonl').open(encoding='utf-8') as stream:
        for row in map(json.loads,stream):
            if row['light']!='green':continue
            if start is None:start=row['time']
            if prev and len(crossings)<8:
                goal=targets[len(crossings)]
                if prev['progress_m']<goal<=row['progress_m']:
                    f=(goal-prev['progress_m'])/(row['progress_m']-prev['progress_m'])
                    crossings.append(prev['time']+f*(row['time']-prev['time']))
            prev=row
            if len(crossings)==8:break
    reasons=Counter(); gains=[];calls=0
    with (folder/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        for rec in map(json.loads,stream):
            if rec.get('kind')!='call':continue
            t=rec['data']['input']['reference_s']
            if start is None or t<start or len(crossings)==8 and t>crossings[-1]:continue
            v,_,detail=rec['data']['output'];calls+=1
            if 'turn_preview' in detail:
                reasons[detail['turn_preview']]+=1
                gains.append(v-detail['constant_arc_target_mps'])
    ends=[start,*crossings]
    return dict(run=folder.name,finish=len(crossings)==8,passed=report['passed'],
                length_m=length,sector_s=[b-a for a,b in zip(ends,ends[1:])],
                body_budget=summarize(folder) if len(crossings)==8 else None,
                calls=calls,preview=dict(reasons),positive_gain_calls=sum(g>1e-6 for g in gains),
                maximum_gain_mps=max(gains,default=0.))


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',choices=('run1','run2','run3','run4'),default='run1')
    selected=parser.parse_args().run
    reference=BASE.parent/'official_prelap_v1/run2' if selected in ('run3','run4') else PREVIOUS/'solo_2p0'
    prior=lap(reference);current=lap(BASE/selected)
    assert math.isclose(prior['length_m'],current['length_m'],abs_tol=.001)
    with (BASE/(selected+'_lap_compare.json')).open('x',encoding='utf-8') as f:
        json.dump(dict(prior=prior, candidate=current,
            sector_time_change_s=[v-u for u,v in zip(prior['sector_s'],current['sector_s'])],
            scope='시작 녹색부터 공식 거리 1/8씩 최초 통과. 서로 다른 단독 주행 1회씩이며 실차 아님.'),
            f,ensure_ascii=False,indent=2)
