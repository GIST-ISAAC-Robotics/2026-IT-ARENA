"""관측만 쓰는 판단을 닫은 뒤 별도 평가 표와 대조. 입구 제거는 반례 입력에만 적용."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig,MotionUnavailable
from intent import BranchIntent

HERE=Path(__file__).resolve().parent
OBS=HERE.parent/'p22_sensor_preview_v1'
ROOT=HERE.parents[3]
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def data(name):
    op=OBS/f'observations_{name}.json'
    obs=json.loads(op.read_bytes())
    src=ROOT/obs['source']; tp=src.parent/'trace.json'
    hashes={str(p.relative_to(ROOT)):digest(p) for p in (op,src,tp)}
    assert digest(src)==obs['sensor_sha256']
    # 범용 기록의 마커 상태만 읽는다. pose, s, actor 정보는 판단에 없다.
    trace=json.loads(tp.read_bytes()); frames=[]
    for q in trace:
        au=q['autonomy'];v=au.get('vision') or {};lr=v.get('last_result') or {}
        frames.append((q.get('time',q.get('t')),au.get('marker_ids',[]),lr.get('stamp'),v.get('epoch')))
    frames.sort(key=lambda f:f[0])
    index={r['t']:r for r in obs['rows']};out=[]
    with src.open() as stream:
        for line in stream:
            e=json.loads(line);d=e['data']
            if e['kind']=='header':cfg=MotionConfig(**d['motion_config'])
            if e['kind']!='call':continue
            t=d['input']['reference_s']
            row=index.pop(t,None)
            if row is None:continue
            raw,_,h=restore_call(d['input'],cfg)
            motion=None
            if out and raw['epoch']==out[-1]['epoch']:
                try:
                    (a,b),_=h.poses([out[-1]['t'],t],t)
                    dx,dy=b[0]-a[0],b[1]-a[1]
                    c,s=math.cos(a[2]),math.sin(a[2])
                    motion=(c*dx+s*dy,-s*dx+c*dy,b[2]-a[2])
                except MotionUnavailable:pass
            stamp=row['scan_stamp'];out.append(dict(t=t,source=stamp[0]+stamp[1]*1e-9,epoch=raw['epoch'],
                                          notches=row['right']['notches'],motion=motion))
    assert not index and all(digest(ROOT/p)==h for p,h in hashes.items())
    return out,frames,hashes

def replay(rows,frames,*,proxy=False,no_marker=False,drop=None):
    machine=BranchIntent();i=0;proposals=[];events=[];cam_epoch=None
    for row in rows:
        t=row['t'];old=(machine.state,machine.reason)
        if row['epoch']!=machine.epoch:machine.reset(t,row['epoch'])
        while i<len(frames) and frames[i][0]<=t:
            observed,ids,stamp,ve=frames[i];i+=1
            if ve is not None and cam_epoch is not None and cam_epoch!=ve:machine.reset(observed,row['epoch'])
            if ve is not None:cam_epoch=ve
            if no_marker:ids=[x for x in ids if x!=20]
            # r27 상태 시각은 원 영상 시각이 아니므로 가정 비교에서만 대입.
            machine.frame(ids,observed if proxy and stamp is None else stamp,observed,row['epoch'])
        n=[] if drop is not None and t in drop else row['notches']
        if machine.scan(t,row['source'],n,row['motion'],row['epoch']):proposals.append(t)
        if old!=(machine.state,machine.reason):
            events.append(dict(t=t,state=machine.state,reason=machine.reason,relative_yaw_deg=math.degrees(machine.yaw)))
    return dict(proposals=proposals,events=events)

def main():
    results={};ok=True
    for name in ('r11','r27'):
        rows,frames,hashes=data(name)
        modes=['strict'] if name=='r11' else ['strict','status_clock_proxy']
        baseline={m:replay(rows,frames,proxy=m!='strict') for m in modes}
        # 사후 평가 및 반례 교란 마스크. 최초 온전한 판단은 이 파일을 열기 전에 확정한다.
        ev=json.loads((OBS/'evaluation.json').read_bytes())['runs'][name]
        exit_t={a['t'] for a in ev['right_notch_exit']}
        # 보존해야 할 출구의 모든 관측을 제외한 실제 간극; 두 원본에서 입구/출구 외 반환 0.
        drop={r['t'] for r in rows if r['notches'] and r['t'] not in exit_t}
        assert len(drop)==ev['right_notch_by_match']['P22_entry']
        pre={a['t'] for a in ev['entry_notch_scans']}
        scored={}
        for m,base in baseline.items():
            proxy=m!='strict'
            no20=replay(rows,frames,proxy=proxy,no_marker=True)
            absent=replay(rows,frames,proxy=proxy,drop=drop)
            cnt=int(name=='r11' or proxy)
            checks=dict(normal_expected_count=len(base['proposals'])==cnt,
                normal_only_before_entry=all(t in pre for t in base['proposals']),
                missing_id_no_proposal=not no20['proposals'],missing_entrance_no_exit_proposal=not absent['proposals'])
            ok=ok and all(checks.values())
            scored[m]=dict(normal=base,without_id20=no20,without_entrance=absent,checks=checks)
        results[name]=dict(inputs=hashes,scans=len(rows),entry_scans_removed=len(drop),exit_scans_preserved=len(exit_t),modes=scored)
    (HERE/'summary.json').write_text(json.dumps(dict(checks_pass=ok,runs=results),indent=2)+'\n',encoding='utf8')
    for n,r in results.items():
        for m,k in r['modes'].items():print(n,m,k['normal']['proposals'],k['checks'])
    assert ok
if __name__=='__main__':main()
