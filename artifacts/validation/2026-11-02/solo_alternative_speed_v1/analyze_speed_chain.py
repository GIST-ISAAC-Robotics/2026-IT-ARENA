"""저장 입력으로 부모 노드의 증속/곡률 제한을 재구성한다. 실행 명령을 보내지 않는다."""
from pathlib import Path
from hashlib import sha256
from zipfile import ZipFile
import json
import math

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
RUN=HERE/'run1'


def analyze():
    report=json.loads((RUN/'report.json').read_bytes())
    trace=json.loads((RUN/'trace.json').read_bytes())
    mcu=json.loads((RUN/'actuation_trace.json').read_bytes())
    start=report['start_time_sim_s']
    stop=next(row['now_us']*1e-6 for row in mcu if row.get('reason')=='software_stop')
    source_names=['src/arena_autonomy/arena_autonomy/wall_follow.py',
                  'src/arena_bringup/launch/simulation.launch.py']
    with ZipFile(RUN/'source_snapshot.zip') as archive:
        for name in source_names:
            assert archive.read(name)==(ROOT/name).read_bytes(), '시험 당시 연산/프로필을 다시 확인할 것'
    assert report['speed_profile']=='g1_open_10kmh_solo_gap05_arc_preview_bridge'
    previous=0.;commands={};curvature_caps=0
    for line in (RUN/'follow_decisions.jsonl').open(encoding='utf-8'):
        event=json.loads(line)
        if event['kind']=='header':
            config=event['data']['follow_config']
            assert config['wheelbase_m']==.135 and config['lateral_acceleration_mps2']==3.
        if event['kind']!='call':
            continue
        data=event['data'];raw,steer=data['output'][:2]
        curvature=abs(math.tan(steer)/.135)
        capped=min(raw, math.sqrt(3./curvature)) if curvature>1e-6 else raw
        curvature_caps+=int(capped<raw-1e-10)
        previous=min(capped,previous+1.5*data['input']['dt_s'])
        stamp=data['input']['reference_s']
        commands[round(stamp,7)]=dict(t=stamp,raw_mps=raw,predicted_mps=previous)
    errors={}
    for row in trace:
        state=row.get('autonomy',{});stamp=round(state.get('sim_time_s',-1),7)
        if stamp in commands:
            errors[stamp]=commands[stamp]['predicted_mps']-state['speed_command_mps']
    window=[r for r in commands.values() if start<=r['t']<stop]
    result=dict(scope='저장 입력의 부모 노드 연산 재구성. 동일 sim_time 상태와 대조하며 STOP 후 명령 제외. 새 주행/실차/모든 비동기 메시지 지연 검증 아님.',
        window_sim_s=[start,stop],calls_in_window=len(window),
        raw_maximum_mps=max(r['raw_mps'] for r in window),
        parent_reconstructed_maximum_mps=max(r['predicted_mps'] for r in window),
        mcu_observed_maximum_mps=max(r['target_speed_mps'] for r in mcu if start<=r['now_us']*1e-6<stop),
        parent_all_calls_maximum_mps=max(r['predicted_mps'] for r in commands.values()),
        matched_status_samples=len(errors),maximum_status_error_mps=max(abs(v) for v in errors.values()),
        curvature_capped_calls=curvature_caps,
        raw_above_2p5_sample_hold_s=.02*sum(r['raw_mps']>=2.5 for r in window),
        parent_above_2p5_sample_hold_s=.02*sum(r['predicted_mps']>=2.5 for r in window),
        sources={name:sha256((ROOT/name).read_bytes()).hexdigest() for name in source_names},
        inputs={name:sha256((RUN/name).read_bytes()).hexdigest() for name in
                ('report.json','trace.json','follow_decisions.jsonl','actuation_trace.json')})
    return result


if __name__=='__main__':
    result=analyze()
    with (HERE/'speed_chain_aligned.json').open('x',encoding='utf-8') as out:
        json.dump(result,out,ensure_ascii=False,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in ('inputs','sources')},ensure_ascii=False))
