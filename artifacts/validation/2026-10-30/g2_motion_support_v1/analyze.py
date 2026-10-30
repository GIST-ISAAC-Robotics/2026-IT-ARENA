"""저장 센서의 실제 반환 분포를 비교한다. 제어 출력을 바꾸지 않는다."""
from collections import Counter, deque
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.scene_motion import motion_scene, transport_points
from arena_autonomy.bypass_handoff import front_patch

SOURCE = BASE.parent/'g2_moving_reserve_v1'
CASES = [('closed_motion_v1', c) for c in ('right_ideal','moving_combined',
           'stopping_combined','stop_restart_combined')]+[('static_motion_v1','right_combined')]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(folder, case):
    path = SOURCE/folder/f'{case}__observed_entry_space_v1.jsonl'
    original_hash = digest(path)
    rows = []; cloud_rows = deque(); seen = None; footer = None; seq = -1
    counts = Counter()
    for line in path.open(encoding='utf-8'):
        event = json.loads(line)
        if footer is not None or event['seq'] != seq+1:
            raise ValueError('input sequence mismatch')
        seq = event['seq']; d = event['data']
        if event['kind'] == 'header':
            if d.get('schema') != 905: raise ValueError('expected candidate record')
            config = MotionConfig(**d['motion_config'])
        elif event['kind'] == 'reset':
            seen = None; cloud_rows.clear()
        elif event['kind'] == 'footer':
            footer = d
        elif event['kind'] == 'call':
            counts['calls'] += 1
            inp = d['input']; scan_data = inp['scan']
            key = (inp['epoch'],scan_data['frame_id'],scan_data['sec'],scan_data['nanosec'])
            if key == seen: continue
            seen = key; counts['distinct_calls'] += 1
            restored, scan, history = restore_call(inp, config)
            try:
                scene = motion_scene(scan,history,restored['reference_s'],
                    timing_verified=restored['timing_verified'],epoch=restored['epoch'],
                    rear_half_angle_deg=restored['rear_half_angle_deg'])
                points, reason = front_patch(scene,'sparse_returns_v1')
            except (ValueError,MotionUnavailable):
                cloud_rows.clear(); counts['invalid_scene'] += 1; continue
            detail = d['output'][2]
            motion = detail.get('follow_entry_space',{}).get('reason') == 'observed_motion_no_entry_reserve'
            record = dict(t=scene.t_ref_s,source_s=scene.source_first_s,
                phase=('stationary' if case.startswith('right') else
                    ('moving' if case=='moving_combined' or scene.t_ref_s<2.5 else
                     'moving_again' if case=='stop_restart_combined' and scene.t_ref_s>=4.0 else 'stopped')),
                motion_release=motion,prepared=detail.get('preparation',{}).get('reason'),
                ready=detail.get('persistence',{}).get('ready'), reason=reason, points=[], comparisons=[])
            # phase는 결과 분석용 각본 시각이며 어떤 제어 입력에도 전달하지 않는다.
            if points is None:
                cloud_rows.clear(); rows.append(record); continue
            record['points'] = points.tolist()
            record['x_quantiles_m'] = np.quantile(points[:,0],[0,.1,.5,.9,1]).tolist()
            while cloud_rows and scene.source_first_s-cloud_rows[0]['source'] > .81:
                cloud_rows.popleft()
            for lag in (.2,.4,.6,.8):
                if not cloud_rows: continue
                old = min(cloud_rows,key=lambda row:abs(scene.source_first_s-row['source']-lag))
                dt = scene.source_first_s-old['source']
                if abs(dt-lag) > .025: continue
                projected = transport_points(old['points'],history,old['time'],scene.t_ref_s)
                distances = np.linalg.norm(points[:,None,:]-projected[None,:,:],axis=2)
                delta = np.median(points,axis=0)-np.median(projected,axis=0)
                record['comparisons'].append(dict(lag_s=dt,old_count=len(projected),current_count=len(points),
                    median_delta_xy_m=delta.tolist(),median_norm_m=float(np.linalg.norm(delta)),
                    front10_delta_x_m=float(np.quantile(points[:,0],.1)-np.quantile(projected[:,0],.1)),
                    current_stationary_support=int(np.count_nonzero(np.min(distances,axis=1)<=.06)),
                    old_stationary_support=int(np.count_nonzero(np.min(distances,axis=0)<=.06)),
                    current_nearest_quantiles_m=np.quantile(np.min(distances,axis=1),[0,.5,1]).tolist()))
            cloud_rows.append(dict(source=scene.source_first_s,time=scene.t_ref_s,points=points.copy()))
            rows.append(record)
        else: raise ValueError('unknown record event')
    if footer is None or any(footer.get(k)!=0 for k in ('capture_errors','failed','dropped','pending')):
        raise ValueError('invalid footer')
    if footer.get('worker_alive') is not False or footer.get('written')!=seq or footer.get('accepted')!=seq:
        raise ValueError('incomplete footer')
    if digest(path)!=original_hash: raise ValueError('input changed')
    phases = {}
    for phase in sorted({r['phase'] for r in rows}):
        group = [r for r in rows if r['phase']==phase]
        release = [r for r in group if r['motion_release']]
        phases[phase] = dict(distinct_calls=len(group),release_calls=len(release),
            release_with_stationary_support=sum(any(c['current_stationary_support']>=2
                and c['old_stationary_support']>=2 and c['median_norm_m']>.06
                for c in r['comparisons']) for r in release))
    return dict(case=case,input_sha256=original_hash,counts=dict(counts),phases=phases,rows=rows)


if __name__ == '__main__':
    out = BASE/'analysis1';out.mkdir(exist_ok=False)
    sources = list((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))+[Path(__file__)]
    hashes = {p.relative_to(ROOT).as_posix():digest(p) for p in sources}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in sources: archive.write(p,p.relative_to(ROOT))
    results = []
    for folder,case in CASES:
        result = analyze(folder,case)
        (out/f'{case}.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        small = {k:v for k,v in result.items() if k!='rows'};results.append(small)
        print(json.dumps(small),flush=True)
    report = dict(results=results,source_sha256=hashes,
        source_unchanged=all(digest(ROOT/p)==h for p,h in hashes.items()),
        scope='stored sensor geometry analysis only; no new control output or object identity proof')
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    if not report['source_unchanged']: raise ValueError('source changed')
