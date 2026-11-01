"""평가 전용: 10 Hz 요약의 끝이 STOP 시각보다 먼저 닫힌 구간을 저장된 고밀도 차체 자세로 독립 확인."""
from bisect import bisect_left, bisect_right
from collections import deque
from hashlib import sha256
from io import BytesIO
import json
from pathlib import Path
import sys
from zipfile import ZipFile

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from g1_traffic_eval import POSE_MAX_GAP_S, Centerline, progress_series


def widest_time(rows, span):
    # 현행 5 cm 범위를 사용하되, 2.5초 미만도 0초로 표시하지 않고 실제 최장 시간을 구한다.
    lows, highs = deque(), deque()
    lo = 0
    best = (0., 0, 0)
    for hi, row in enumerate(rows):
        p = row['progress_m']
        while lows and rows[lows[-1]]['progress_m'] >= p: lows.pop()
        while highs and rows[highs[-1]]['progress_m'] <= p: highs.pop()
        lows.append(hi); highs.append(hi)
        while rows[highs[0]]['progress_m'] - rows[lows[0]]['progress_m'] > span:
            lo += 1
            if lows[0] < lo: lows.popleft()
            if highs[0] < lo: highs.popleft()
        duration = row['t'] - rows[lo]['t']
        if duration > best[0]: best = (duration, lo, hi)
    dt, left, right = best
    return dict(duration_s=dt, from_s=rows[left]['t'], to_s=rows[right]['t'],
                actual_progress_span_m=float(np.ptp([r['progress_m'] for r in rows[left:right+1]])))


def audit(run):
    report = json.loads((run/'report.json').read_bytes())
    first, last = report['green_observed_sim_s'], report['stop_request_sim_s']
    members = {k:v for k,v in report['source_sha256'].items()
               if k.startswith('artifacts/validation/2026-11-01/signal_marker_offset_v1/fixture/')
               and k.endswith(('centerline.csv','scene.json'))}
    if len(members) != 2: raise ValueError('시험 당시 중심선과 scene이 필요함')
    with ZipFile(run/'source_snapshot.zip') as z:
        data = {}
        for k,v in members.items():
            raw = z.read(k)
            if sha256(raw).hexdigest() != v: raise ValueError(k)
            data[Path(k).name] = raw
        evaluator='scripts/g1_traffic_eval.py'
        if z.read(evaluator)!=(ROOT/evaluator).read_bytes(): raise ValueError('평가기 버전 다름')
    route=np.genfromtxt(BytesIO(data['centerline.csv']), delimiter=',',names=True)
    length=json.loads(data['scene.json'])['track']['lap_length_m']
    line=Centerline(route['x_m'],route['y_m'],route['s_m'],length)
    raw=(run/'ego_poses.jsonl').read_bytes()
    poses=[tuple(json.loads(a)) for a in raw.splitlines()]
    if any(len(p)!=4 or not all(np.isfinite(p)) for p in poses): raise ValueError('invalid pose')
    ts=[p[0] for p in poses]
    if not (ts[0]<=first<last<=ts[-1]) or np.min(np.diff(ts))<=0: raise ValueError('window not covered')
    # 경계 보간 전후의 원시 시각을 모두 보존하고, 그 사이 최대 간격도 검사한다.
    i,j=bisect_right(ts,first)-1,bisect_left(ts,last)
    gap=max(np.diff(ts[i:j+1]))
    if gap > POSE_MAX_GAP_S+1e-9: raise ValueError('uncovered dense pose interval')
    def edge(at):
        k=bisect_left(ts,at)
        if ts[k]==at:return poses[k]
        a,b=poses[k-1:k+1];f=(at-a[0])/(b[0]-a[0])
        # yaw는 중심선 진행 계산에 쓰이지 않는다.
        return (at,a[1]+f*(b[1]-a[1]),a[2]+f*(b[2]-a[2]),a[3])
    window=[edge(first),*poses[bisect_right(ts,first):bisect_left(ts,last)],edge(last)]
    rows,jumps=progress_series(line,window)
    if jumps: raise ValueError('centerline projection jump')
    result=widest_time(rows,.05)
    return dict(scope='기존 10 Hz 진행 요약 실패 보존. 평가 전용 고밀도 자세의 중심선 5 cm 범위 최장 지속; 실차/공식 벌점 아님.',
        inputs={'report.json':sha256((run/'report.json').read_bytes()).hexdigest(),
                'ego_poses.jsonl':sha256(raw).hexdigest(), **members},
        window=[first,last],coverage=dict(bracket=[ts[i],ts[j]],maximum_gap_s=gap,samples=len(rows)),
        max_nonprogress=result, projection_jumps=jumps,
        below_10s=result['duration_s']<10., progress_m=rows[-1]['progress_m'])


if __name__=='__main__':
    r=audit(HERE/'run1')
    with (HERE/'run1_dense_progress.json').open('x',encoding='utf-8') as f:
        json.dump(r,f,ensure_ascii=False,indent=2)
    print(json.dumps(r['max_nonprogress']))
