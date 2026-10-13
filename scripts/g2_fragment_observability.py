#!/usr/bin/env python3
"""정지 표적의 관측 조각/보이는 점 중심 이동 진단. 실제 추적기는 아니다."""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import zipfile

from arena_autonomy.observed_fragments import partition_observed
from g2_geometry_probe import ROOT
from g2_visibility_probe import sequential_scene


def perturb(scene, range_bias_m=0., hide_indices=()):
    """±3 cm 탐색 오차/결측만 주입하며 관측 시각·방향·원점을 유지한다."""
    if not math.isfinite(range_bias_m):
        raise ValueError('finite range bias required')
    hidden = set(hide_indices)
    changed=[]
    for ray in scene.rays:
        if ray.status != 'return':
            changed.append(ray)
        elif ray.source_index in hidden:
            changed.append(replace(ray,status='unknown',hit_xy_m=None))
        else:
            distance=math.dist(ray.ray_origin_xy_m,ray.hit_xy_m)+range_bias_m
            if not .05 <= distance <= 12.:
                changed.append(replace(ray,status='unknown',hit_xy_m=None))
            else:
                endpoint=tuple(ray.ray_origin_xy_m[i]+distance*ray.direction_xy[i] for i in (0,1))
                changed.append(replace(ray,hit_xy_m=endpoint))
    return replace(scene,rays=tuple(changed))


def world_point(point, reference):
    x,y,yaw=reference
    c,s=math.cos(yaw),math.sin(yaw)
    return x+c*point[0]-s*point[1],y+s*point[0]+c*point[1]


def summarize(scene, labels, reference):
    if len(labels)!=len(scene.rays):
        raise ValueError('truth labels must align with rays for this evaluator')
    truth={ray.source_index:label for ray,label in zip(scene.rays,labels)}
    result=partition_observed(scene)
    target_rays=[r for r in scene.rays if r.status=='return' and truth[r.source_index]=='target']
    containing=[]
    for i,fragment in enumerate(result.fragments):
        labels_in_fragment=Counter(truth[r.source_index] for r in fragment.rays)
        if labels_in_fragment['target']:
            containing.append(dict(index=i,points=len(fragment.rays),labels=dict(labels_in_fragment),
                contaminated=len(labels_in_fragment)>1,source_first_s=fragment.source_first_s,
                source_last_s=fragment.source_last_s))
    # 정답 label로 모은 반환 중심: 실제 군집기보다 유리한 oracle 진단이다.
    # 조각 하나가 한 물체라는 가정 없이 부분 표면 중심의 문제만 분리한다.
    if target_rays:
        points=[world_point(r.hit_xy_m,reference) for r in target_rays]
        centroid=tuple(sum(p[i] for p in points)/len(points) for i in (0,1))
        stamp=sum(r.acquired_at_s for r in target_rays)/len(target_rays)
    else:
        centroid=stamp=None
    return dict(reference_s=scene.t_ref_s,source_first_s=scene.source_first_s,
        source_last_s=scene.source_last_s,fragment_count=len(result.fragments),
        target_returns=len(target_rays),target_containing_fragments=containing,
        target_fragment_count=len(containing),contaminated_fragments=sum(f['contaminated'] for f in containing),
        oracle_visible_centroid_world=centroid,oracle_mean_acquisition_s=stamp,
        oracle_center_error_m=math.dist(centroid,(1.5,.24)) if centroid else None,
        driving_authorized=False)


def apparent_velocities(rows):
    pairs=[]
    for previous,current in zip(rows,rows[1:]):
        a,b=previous['oracle_visible_centroid_world'],current['oracle_visible_centroid_world']
        if a is None or b is None:
            continue
        dt=current['oracle_mean_acquisition_s']-previous['oracle_mean_acquisition_s']
        if dt <= 0:
            raise ValueError('centroid acquisition time did not progress')
        pairs.append(dict(from_reference_s=previous['reference_s'],to_reference_s=current['reference_s'],
            mean_acquisition_dt_s=dt,speed_mps=math.dist(a,b)/dt))
    return pairs


def evaluate(rows, height, mode):
    if mode not in ('clean','alternating_3cm','missing_every_seventh'):
        raise ValueError('unknown diagnostic mode')
    stages=[]
    for i in range(1,int(rows[-1]['t_s']/.1)+1):
        scene,labels,reference=sequential_scene(rows,i*.1,height)
        if mode=='alternating_3cm':
            scene=perturb(scene,.03 if i%2 else -.03)
        elif mode=='missing_every_seventh':
            scene=perturb(scene,hide_indices=range(i%7,500,7))
        stages.append(summarize(scene,labels,reference))
    speeds=apparent_velocities(stages)
    errors=[s['oracle_center_error_m'] for s in stages if s['oracle_center_error_m'] is not None]
    return dict(height_m=height,mode=mode,scans=len(stages),stages=stages,
        target_visible_scans=sum(s['target_returns']>0 for s in stages),
        split_scans=sum(s['target_fragment_count']>1 for s in stages),
        contaminated_scans=sum(s['contaminated_fragments']>0 for s in stages),
        center_error_max_m=max(errors,default=None),apparent_speed_max_mps=max((p['speed_mps'] for p in speeds),default=None),
        apparent_velocity_pairs=speeds,truth_target_speed_mps=0.,driving_authorized=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    output=args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('new validation subdirectory required')
    data=args.trace.read_bytes()
    rows=json.loads(data)
    output.mkdir(parents=True,exist_ok=False)
    names=['scripts/g2_fragment_observability.py','scripts/g2_visibility_probe.py',
        'scripts/g2_tracking_probe.py','scripts/g2_geometry_probe.py','scripts/g0_geometry.py',
        'tests/test_g2_fragment_observability.py',
        'src/arena_autonomy/arena_autonomy/observed_fragments.py',
        'src/arena_autonomy/arena_autonomy/local_scene.py']
    sources={name:(ROOT/name).read_bytes() for name in names}
    with zipfile.ZipFile(output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as bundle:
        for name,value in sources.items(): bundle.writestr(name,value)
    results=[evaluate(rows,height,mode) for height in (.12,.06)
             for mode in ('clean','alternating_3cm','missing_every_seventh')]
    report=dict(scope='oracle_label_static_object_visible_surface_diagnostic_only',results=results,
        source_sha256={name:hashlib.sha256(value).hexdigest() for name,value in sources.items()},
        sources_unchanged=all((ROOT/name).read_bytes()==value for name,value in sources.items()),
        input_sha256=hashlib.sha256(data).hexdigest(),input_unchanged=args.trace.read_bytes()==data,
        driving_authorized=False, limits=[
            'Truth ego transform and target labels are evaluation-only, not tracking inputs.',
            'Visible-surface centroid is not the object center; no association/velocity estimator is implemented.',
            'Alternating scan-wide 3 cm bias and deterministic missing rays are stress inputs, not C1 error distributions.',
            'No-observation metrics are null, not zero speed or obstacle absence.',
            'Fragment thresholds are provisional; groups do not imply complete objects.'])
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps([{k:v for k,v in r.items() if k not in ('stages','apparent_velocity_pairs')} for r in results]))
    return 0 if report['sources_unchanged'] and report['input_unchanged'] else 1


if __name__=='__main__':
    raise SystemExit(main())
