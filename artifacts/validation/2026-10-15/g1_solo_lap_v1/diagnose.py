"""실패 기록의 경계 관측을 그린다. 제어기/표본/판정선을 수정하지 않는다."""
import hashlib
import json
from pathlib import Path
import zipfile
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.boundary_evidence import fragment_evidence
from audit_local_pursuit_road import road_polygon
from build_experimental_track import rectangle

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''): h.update(block)
    return h.hexdigest()


def main():
    output=BASE/'diagnosis.json'
    if output.exists(): raise FileExistsError(output)
    run=BASE/'run'
    report=json.loads((run/'report.json').read_text())
    sources={p:digest(ROOT/p) for p in report['input_sha256']}
    if sources!=report['input_sha256']: raise ValueError('run source changed')
    inputs=['follow_decisions.jsonl','ego_poses.jsonl','report.json','runtime_world.sdf']
    hashes={p:digest(run/p) for p in inputs}
    with zipfile.ZipFile(BASE/'diagnosis_source.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for path in [Path(__file__),ROOT/'scripts/analyze_follow_trace.py',ROOT/'scripts/audit_local_pursuit_road.py']:
            archive.write(path,path.relative_to(ROOT))
    selected=[1591,1596,1601,1851]
    found=[]
    with (run/'follow_decisions.jsonl').open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='header':
                motion=MotionConfig(**event['data']['motion_config'])
                config=FollowConfig(**event['data']['follow_config'])
            elif event['kind']=='call' and event['seq'] in selected: found.append(event)
    fig,axes=plt.subplots(2,2,figsize=(12,9),constrained_layout=True)
    rows=[]
    for event,ax in zip(found,axes.ravel()):
        data,scan,history=restore_call(event['data']['input'],motion)
        scene=motion_scene(scan,history,data['reference_s'],timing_verified=True,
                           epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
        points=np.asarray(scene.occupied_endpoints_xy_m)
        ax.scatter(points[:,0],points[:,1],s=6,color='#485566',label='All occupied returns')
        candidates={}
        right=None
        for side,color in [('left','#d83c3c'),('right','#1477c9')]:
            path,info=local_path(points,side,config.wall_offset_m)
            candidates[side]={k:v for k,v in info.items() if k!='path_points'}
            if path is not None:
                ax.plot(path[:,0],path[:,1],color=color,lw=2,label=side+' path candidate')
            if side=='right': right=path
        evidence=fragment_evidence(points,right,'left',config) if right is not None else None
        rows.append(dict(seq=event['seq'],t=data['reference_s'],recorded_output=event['data']['output'],
                         candidates=candidates,independent_fragment_recompute=evidence))
        ax.add_patch(plt.Rectangle((-.1,-.075),.2,.15,color='#333',alpha=.35))
        ax.set(xlim=(-.4,2.),ylim=(-1.,1.),xlabel='Forward x [m]',ylabel='Left y [m]',
               title=f"seq {event['seq']}, t={data['reference_s']:.2f} s")
        ax.set_aspect('equal'); ax.grid(alpha=.25); ax.legend(fontsize=8)
    fig.suptitle('Stored sensor-only observations: boundary loss, no extra objects or synthetic points')
    fig.savefig(BASE/'boundary_loss.png',dpi=150)
    # 별도 평가 정답으로, 실패 STOP 이후의 도로 감사 공백도 확인한다.
    road=road_polygon(run/'runtime_world.sdf')
    poses=[json.loads(line) for line in (run/'ego_poses.jsonl').read_text().splitlines()]
    first=report['failure_cleanup_stop']['dense_pose_evidence']['required_first_s']
    last=report['failure_cleanup_stop']['dense_pose_evidence']['required_last_s']
    stop=[p for p in poses if first<=p[0]<=last]
    outside=[rectangle(p[1],p[2],p[3],.2,.15).difference(road).area for p in stop]
    result=dict(scope='saved_input_diagnosis_not_fix_or_new_drive',rows=rows,
                failure_stop_road=dict(window=[first,last],samples=len(stop),
                    outside_over_1cm2=sum(a>.0001 for a in outside),maximum_area_m2=max(outside,default=None),
                    scope='sampled body footprint only; not continuous wheel/official penalty proof'),
                input_sha256=hashes,input_unchanged=all(digest(run/p)==h for p,h in hashes.items()),
                source_unchanged=all(digest(ROOT/p)==h for p,h in sources.items()),
                original_verdict_changed=False,driving_authorized=False)
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({**{k:v for k,v in result.items() if k!='rows'},
        'selected':[dict(seq=r['seq'],t=r['t'],reason=r['recorded_output'][2]['reason'],
             candidates=r['candidates'],fragment_reason=r['independent_fragment_recompute']['reason'],
             observed_ahead_sum_m=r['independent_fragment_recompute']['observed_ahead_sum_m']) for r in rows]},ensure_ascii=False))


if __name__=='__main__': main()
