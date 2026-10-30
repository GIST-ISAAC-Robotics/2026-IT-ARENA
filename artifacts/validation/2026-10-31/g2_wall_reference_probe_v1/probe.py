"""저장 센서의 기존 직선 기준과 현재 연결 반환 재적합 비교. 주행 미연결."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile
from collections import Counter
import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.bypass_entry_trial import straight_reference
from arena_autonomy.bypass_wall_fit import tracked_straight_wall
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene,transport_points


def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def paired(fits,config):
    if any(f is None for f in fits):return None,'current_side_unavailable'
    a,b=fits;lo,hi=max(a[2],b[2]),min(a[3],b[3])
    if lo>-config.wheelbase_m/2 or hi-max(lo,0.)<.25:return None,'reference_does_not_cover_entry'
    difference=max(abs((a[0]-b[0])*x+a[1]-b[1]) for x in (lo,hi))
    if difference>config.boundary_agreement_m:return None,'observed_wall_reference_disagrees'
    return ((a[0]+b[0])/2,(a[1]+b[1])/2,lo,hi),'current_tracked_pair'


def line_of(fit):
    a,b,lo,hi=fit
    return np.array([[lo,a*lo+b],[hi,a*hi+b]])


def inspect_trace(path,output):
    before=sha(path);anchors={};last_key=None;counts=Counter();reasons=Counter();samples=[]
    with path.open() as stream:
        for event in map(json.loads,stream):
            if event['kind']=='header':
                header=event['data'];motion=MotionConfig(**header['motion_config']);config=FollowConfig(**header['follow_config'])
            elif event['kind']=='reset':anchors={};last_key=None
            elif event['kind']=='call':
                inp=event['data']['input'];scan=inp['scan']
                key=(scan['frame_id'],inp['epoch'],scan['sec']+scan['nanosec']*1e-9)
                if key==last_key:continue
                if last_key is not None and (key[:2]!=last_key[:2] or not 0<key[2]-last_key[2]<=.25):anchors={}
                last_key=key
                data,raw,history=restore_call(inp,motion);now=data['reference_s']
                try:
                    scene=motion_scene(raw,history,now,timing_verified=data['timing_verified'],
                        epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
                except ValueError:
                    anchors={};counts['invalid_scene']+=1;continue
                original,detail=straight_reference(scene,config)
                fits=[];tracked={}
                for side in ('left','right'):
                    old=anchors.get(side);fit,evidence=None,dict(reason='no_recent_seed')
                    if old is not None and 0<=now-old['time']<=1.:
                        try:
                            line=transport_points(old['line'],history,old['time'],now)
                            fit,evidence=tracked_straight_wall(scene,config,side,line)
                        except ValueError:evidence=dict(reason='seed_transport_unavailable')
                    fits.append(fit);tracked[side]=evidence
                    if fit is not None:anchors[side]=dict(line=line_of(fit),time=now)
                proposed,why=paired(fits,config)
                if original is not None:
                    # 최초 탐색 위치는 원래의 양벽 합의에서만 얻는다. 지도/정답 자세를 쓰지 않는다.
                    for side,wall in detail['walls'].items():
                        lo,hi=wall['observed_x_m']
                        anchors[side]=dict(line=line_of((wall['slope'],wall['intercept_m'],lo,hi)),time=now)
                    counts['original_valid']+=1
                else:
                    counts['original_invalid']+=1;counts['recovered' if proposed is not None else 'still_invalid']+=1
                    reasons[(detail.get('reason'),why)]+=1
                samples.append(freeze(dict(time_s=now,original_reference=original,original=detail,
                    proposed_reference=proposed,proposed_reason=why,current_fits=fits,tracked=tracked,
                    original_output=event['data']['output'][:2],controller_preparation=event['data']['output'][2].get('preparation'))))
    prefix=path.parent.parent.name
    (output/(prefix+'_samples.json')).write_text(json.dumps(samples,indent=2))
    return dict(input=str(path.relative_to(ROOT)),sha256=before,input_unchanged=sha(path)==before,
        unique_scenes=len(samples),counts=dict(counts),reasons=[dict(original=k[0],candidate=k[1],count=v) for k,v in reasons.items()],
        scope='current-return geometry diagnostics; no output/path change or new driving proof')


if __name__=='__main__':
    output=BASE/'run1';output.mkdir(exist_ok=False)
    sources=[Path(__file__),BASE/'PLAN.md',BASE/'run.sh']
    sources.extend((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in sources}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for path in hashes:archive.write(ROOT/path,path)
    cases=['g2_preparation_hold_gazebo_v1','g2_course_trial_v1']
    results=[inspect_trace(BASE.parent/case/'run1/follow_decisions.jsonl',output) for case in cases]
    result=dict(results=results,source_sha256=hashes,source_unchanged=all(sha(ROOT/p)==h for p,h in hashes.items()))
    (output/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(results=results,source_unchanged=result['source_unchanged'])))
