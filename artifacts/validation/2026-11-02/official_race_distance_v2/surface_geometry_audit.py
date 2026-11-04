"""기존 노면 평가의 unary_union 경고 뒤 결과 기하의 유효성만 확인. 재주행/기준 변경 없음."""
import json, math, warnings
from bisect import bisect_right
from pathlib import Path
import sys
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
import surface_contact_eval as surface


def inspect(g):
    return dict(valid=bool(g.is_valid), empty=bool(g.is_empty), area=float(g.area),
                finite=all(math.isfinite(v) for v in (*g.bounds, g.area)))


if __name__ == '__main__':
    with warnings.catch_warnings(record=True) as logged:
        warnings.simplefilter('always', RuntimeWarning)
        world=surface.SurfaceMap.from_world(HERE/'run1/runtime_world.sdf')
    result=dict(scope='기존 전체 노면 계산의 수치 경고 후 기하 유효성 확인. 원시 결과는 수정하지 않음.',
        map_warnings=[str(w.message) for w in logged],
        surfaces={name:inspect(getattr(world,name)) for name in ('road','grass','walls')})
    poses, source=surface.load_poses(HERE/'run1')
    steering=surface.load_steering(HERE/'run1')
    index=[s[0] for s in steering]
    bad=[]; warning_samples=[]
    for t,x,y,yaw in surface.deduplicate(poses):
        i=bisect_right(index,t)-1
        steer=steering[i][1] if i>=0 else 0.
        with warnings.catch_warnings(record=True) as logged:
            warnings.simplefilter('always', RuntimeWarning)
            footprint=surface.steering_footprint(x,y,yaw,steer)
        d=inspect(footprint)
        if not d['valid'] or d['empty'] or not d['finite'] or d['area']<.03-1e-10:
            bad.append(dict(t=t,**d))
        if logged:
            warning_samples.append(dict(t=t,messages=[str(w.message) for w in logged],**d))
    result.update(pose_count=len(poses),bad_footprints=bad,footprint_warning_samples=warning_samples,
        all_valid=not bad and all(d['valid'] and not d['empty'] and d['finite'] for d in result['surfaces'].values()))
    (HERE/'surface_geometry_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False),flush=True)
    raise SystemExit(0 if result['all_valid'] else 1)
