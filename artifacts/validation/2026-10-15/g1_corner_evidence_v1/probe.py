"""네 보존 입력의 새 증거만 비교한다. 새 주행/정답 입력 없음."""
import hashlib
import json
from pathlib import Path
import zipfile
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.corner_evidence import corner_anchor_evidence

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
INPUT=BASE.parent/'g1_solo_lap_v1/run/follow_decisions.jsonl'


def digest(path):
    result=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''): result.update(chunk)
    return result.hexdigest()


def main():
    output=BASE/'report.json'
    if output.exists(): raise FileExistsError(output)
    sources=['src/arena_autonomy/arena_autonomy/'+name+'.py' for name in
        ('corner_evidence','follow_trace','local_follow','local_path','scene_motion','local_scene','lidar_motion')]
    sources+=['tests/test_corner_evidence.py',str(Path(__file__).relative_to(ROOT))]
    hashes={p:digest(ROOT/p) for p in sources}
    before=digest(INPUT)
    with zipfile.ZipFile(BASE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for path in sources: archive.write(ROOT/path,path)
    rows=[]
    with INPUT.open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='header':
                motion=MotionConfig(**event['data']['motion_config'])
                config=FollowConfig(**event['data']['follow_config'])
            elif event['kind']=='call' and event['seq'] in (1591,1596,1601,1851):
                data,scan,history=restore_call(event['data']['input'],motion)
                scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                    epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
                points=np.asarray(scene.occupied_endpoints_xy_m)
                reference,info=local_path(points,'right',config.wall_offset_m)
                rows.append(dict(seq=event['seq'],reference_s=data['reference_s'],
                    original_reason=event['data']['output'][2]['reason'],
                    evidence=corner_anchor_evidence(points,reference,'left',config),
                    opposite_info=info,points=points.tolist()))
    result=dict(scope='saved_sensor_input_corner_evidence_only',rows=rows,input_sha256=before,
        input_unchanged=before==digest(INPUT),source_sha256=hashes,
        source_unchanged=all(digest(ROOT/p)==h for p,h in hashes.items()),driving_authorized=False)
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({**{k:v for k,v in result.items() if k not in ('rows','source_sha256')},
        'rows':[dict(seq=r['seq'],evidence={k:v for k,v in r['evidence'].items() if k!='source_indices'}) for r in rows]}))


if __name__=='__main__': main()
