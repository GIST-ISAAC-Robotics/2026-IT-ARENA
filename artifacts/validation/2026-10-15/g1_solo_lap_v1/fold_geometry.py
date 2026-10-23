"""저장 센서 입력의 기대 벽 역방향 구간 위치만 진단한다. 제어 변경 없음."""
import json
from pathlib import Path
import zipfile
import numpy as np
from diagnose import (BASE, ROOT, digest, restore_call, MotionConfig, FollowConfig,
                      motion_scene, local_path)


def main():
    output = BASE/'fold_geometry.json'
    if output.exists():
        raise FileExistsError(output)
    trace = BASE/'run/follow_decisions.jsonl'
    before = digest(trace)
    source = json.loads((BASE/'run/report.json').read_text())['input_sha256']
    if any(digest(ROOT/p) != h for p, h in source.items()):
        raise ValueError('source differs from drive')
    with zipfile.ZipFile(BASE/'fold_geometry_source.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in (Path(__file__), BASE/'diagnose.py'):
            archive.write(path, path.relative_to(ROOT))
    rows = []
    with trace.open() as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'header':
                motion = MotionConfig(**event['data']['motion_config'])
                config = FollowConfig(**event['data']['follow_config'])
            elif event['kind'] == 'call' and event['seq'] in (1591,1596,1601,1851):
                data, scan, history = restore_call(event['data']['input'], motion)
                scene = motion_scene(scan, history, data['reference_s'], timing_verified=True,
                    epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
                points = np.asarray(scene.occupied_endpoints_xy_m)
                path, _ = local_path(points, 'right', config.wall_offset_m)
                tangent = np.gradient(path, axis=0)
                tangent /= np.linalg.norm(tangent, axis=1)[:,None]
                wall = path + config.wall_offset_m*np.column_stack((-tangent[:,1],tangent[:,0]))
                steps = np.diff(wall, axis=0)
                lengths = np.linalg.norm(steps,axis=1)
                direction = np.sum(steps*(tangent[1:]+tangent[:-1]),axis=1)
                bad = np.flatnonzero((lengths < 1e-7) | (direction <= 0))
                arc = np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
                origin = int(np.argmin(np.linalg.norm(path,axis=1)))
                rows.append(dict(seq=event['seq'], t=data['reference_s'], point_count=len(path),
                    origin_index=origin, wall_offset_m=config.wall_offset_m,
                    folded_segment_indices=bad.tolist(),
                    folded_segments=[dict(index=int(i), path_start_xy_m=path[i].tolist(),
                        path_arc_from_origin_m=float(arc[i]-arc[origin]),
                        expected_wall_start_xy_m=wall[i].tolist(),
                        expected_wall_end_xy_m=wall[i+1].tolist(),
                        forward_dot_m=float(direction[i]), length_m=float(lengths[i])) for i in bad],
                    first_fold_path_ahead_m=float(arc[bad[0]]-arc[origin]) if len(bad) else None,
                    endpoint_only=bool(len(bad) and all(i in (0,len(path)-2) for i in bad))))
    result = dict(scope='saved input geometry; no clipping, threshold change, or drive authorization',
        rows=rows, trace_sha256=before, input_unchanged=digest(trace)==before,
        source_unchanged=all(digest(ROOT/p)==h for p,h in source.items()),
        original_verdict_changed=False, driving_authorized=False)
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({**{k:v for k,v in result.items() if k!='rows'},'rows':[
        {k:v for k,v in row.items() if k!='folded_segments'} for row in rows]},ensure_ascii=False))


if __name__ == '__main__':
    main()
