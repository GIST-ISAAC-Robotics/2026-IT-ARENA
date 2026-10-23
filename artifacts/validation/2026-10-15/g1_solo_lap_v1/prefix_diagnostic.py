"""저장 입력에서 첫 기대 벽 접힘 전 경로만 남기는 단순 대안의 한계. 운영 미연결."""
import json
from pathlib import Path
import zipfile
import numpy as np
from diagnose import (BASE, ROOT, digest, restore_call, MotionConfig, FollowConfig,
                      motion_scene, local_path, fragment_evidence)


def main():
    output = BASE/'prefix_diagnostic.json'
    if output.exists():
        raise FileExistsError(output)
    trace = BASE/'run/follow_decisions.jsonl'
    before = digest(trace)
    sources = json.loads((BASE/'run/report.json').read_text())['input_sha256']
    if any(digest(ROOT/p) != h for p,h in sources.items()):
        raise ValueError('source changed')
    with zipfile.ZipFile(BASE/'prefix_diagnostic_source.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for path in (Path(__file__),BASE/'diagnose.py'):
            archive.write(path,path.relative_to(ROOT))
    rows = []
    with trace.open() as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind']=='header':
                motion=MotionConfig(**event['data']['motion_config'])
                config=FollowConfig(**event['data']['follow_config'])
            elif event['kind']=='call' and event['seq'] in (1591,1596,1601,1851):
                data,scan,history=restore_call(event['data']['input'],motion)
                scene=motion_scene(scan,history,data['reference_s'],timing_verified=True,
                    epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
                points=np.asarray(scene.occupied_endpoints_xy_m)
                path,_=local_path(points,'right',config.wall_offset_m)
                tangent=np.gradient(path,axis=0)
                tangent/=np.linalg.norm(tangent,axis=1)[:,None]
                wall=path+config.wall_offset_m*np.column_stack((-tangent[:,1],tangent[:,0]))
                steps=np.diff(wall,axis=0)
                folded=np.flatnonzero((np.linalg.norm(steps,axis=1)<1e-7) |
                    (np.sum(steps*(tangent[1:]+tangent[:-1]),axis=1)<=0))
                stop=int(folded[0]+1) if len(folded) else len(path)
                # 함수 전체를 그대로 적용하므로 끝점 접힘 재발/길이 부족도 그대로 실패한다.
                result=fragment_evidence(points,path[:stop],'left',config)
                rows.append(dict(seq=event['seq'],t=data['reference_s'],retained_path_points=stop,
                    evidence=result))
    result=dict(scope='offline first-fold prefix alternative; no controller change or drive',rows=rows,
        input_sha256=before,input_unchanged=digest(trace)==before,
        source_unchanged=all(digest(ROOT/p)==h for p,h in sources.items()),
        original_verdict_changed=False,driving_authorized=False)
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({**{k:v for k,v in result.items() if k!='rows'},'rows':[
        dict(seq=r['seq'],retained_path_points=r['retained_path_points'],
            supported=r['evidence']['supported'],reason=r['evidence']['reason'],
            observed_ahead_sum_m=r['evidence']['observed_ahead_sum_m'],
            anchor_observed=r['evidence']['anchor_observed']) for r in rows]}))


if __name__=='__main__':
    main()
