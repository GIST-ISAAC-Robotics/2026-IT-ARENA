#!/usr/bin/env python3
"""저장된 연속 C1 장면의 정적 표면 대응 진단. 제어/주행에는 연결하지 않는다."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import zipfile

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.surface_correspondence import SurfaceBounds, stationary_surface_candidates
from inspect_corner_geometry import components
from profile_follow_trace import SNAPSHOT_MODULES
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def digest(path):
    result=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):result.update(chunk)
    return result.hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--start',type=float,default=41.8)
    parser.add_argument('--end',type=float,default=44.8)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    before=digest(args.input)
    bounds=SurfaceBounds(.03,.01)
    previous=None
    rows=[]
    sources=[f'src/{pkg}/{pkg}/{mod}.py' for pkg,mods in SNAPSHOT_MODULES.items() for mod in mods]
    sources+=['src/arena_autonomy/arena_autonomy/surface_correspondence.py',
              'src/arena_autonomy/arena_autonomy/observed_fragments.py',
              'scripts/probe_surface_correspondence.py','scripts/inspect_corner_geometry.py',
              'scripts/profile_follow_trace.py','scripts/probe_corner_observability.py',
              'tests/test_surface_correspondence.py']
    hashes={p:digest(ROOT/p) for p in sources}
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in sources:z.write(ROOT/p,p)
    with args.input.open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='header':motion=MotionConfig(**event['data']['motion_config'])
            elif event['kind']=='reset':previous=None
            elif event['kind']=='call' and args.start<=event['data']['input']['reference_s']<=args.end:
                data,scan,history=restore_call(event['data']['input'],motion)
                scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                                   epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
                if previous is not None and scene.source_first_s==previous.source_first_s:continue
                if previous is not None:
                    result=stationary_surface_candidates(previous,scene,history,bounds)
                    p=np.asarray(scene.occupied_endpoints_xy_m)
                    parts=components(p,data['side'])
                    sign=1 if data['side']=='left' else -1
                    near=min(parts,key=lambda c:float(np.min(np.linalg.norm(p[c['indices']]-[0,sign*.425],axis=1)))) if parts else None
                    returns=[r for r in scene.rays if r.status=='return']
                    indices=[returns[i].source_index for i in near['indices']] if near else []
                    supported=set(result['current_supported_source_indices'])
                    rows.append(dict(seq=event['seq'],time_s=scene.t_ref_s,
                        near_source_indices=indices,near_points=len(indices),
                        near_supported_points=sum(i in supported for i in indices),
                        recorded_reason=event['data']['output'][2]['reason'],result=result))
                previous=scene
    report=dict(scope='saved_input_static_compatibility_not_identity_or_control',
                bounds_assumptions=asdict(bounds),rows=rows,
                evaluated=sum(r['result']['evaluated'] for r in rows),pairs=len(rows),
                reasons=dict(Counter(r['result']['reason'] for r in rows)),
                input_sha256=before,input_unchanged=before==digest(args.input),source_sha256=hashes,
                source_unchanged=all(digest(ROOT/p)==v for p,v in hashes.items()),
                driving_authorized=False,memory_refresh_authorized=False,runtime_changed=False)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','source_sha256')}))


if __name__=='__main__':main()
