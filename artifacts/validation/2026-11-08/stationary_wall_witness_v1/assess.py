"""실제 우회/복귀 평가와 새 코스 연결·무결성 대조. 원 판정은 보존한다."""
import hashlib
import json
from pathlib import Path
import sys
import yaml
from goal_eval import evaluate

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]


def main(case):
    folder=HERE/case; run=folder/'run1'
    evaluate(run,ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06')
    result=json.loads((folder/'run1_goal.json').read_text())
    report=json.loads((run/'report.json').read_text())
    with (run/'follow_decisions.jsonl').open() as f: header=json.loads(next(f))['data']
    sensor=ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'
    actual=yaml.safe_load((run/'runtime_local_sensors.yaml').read_text())
    expected=yaml.safe_load(sensor.read_text())
    integrity=json.loads((folder/'postflight_integrity_run1.json').read_text())
    signal=report.get('official_signal_evidence',{})
    integration=dict(
        b0390=actual['camera']==expected['camera'] and report['local_sensor_config']['sha256']==hashlib.sha256(sensor.read_bytes()).hexdigest(),
        piecewise=header['follow_config']['piecewise_wall_trial'] is True,
        main_section=header['follow_config']['wall_offset_m']==.505,
        wall_witness=header['execution_config']['bypass'].get('rear_wall_witness') is True,
        slow_profile=header['execution_config'].get('moving_pass_profile','slow_v1')=='slow_v1',
        actual_green=bool(signal.get('passed') and signal.get('start_after_actual_green')),
        source_integrity=not integrity['changed'],
        process_clean=not (folder/'postflight_processes_run1.txt').read_text().strip(),
    )
    result.update(integration=integration,raw=dict(passed=report['passed'],gates=report['gates']))
    result['supplementary_all_true']=result['passed'] and all(integration.values())
    (folder/'assessment_run1.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(supplementary_all_true=result['supplementary_all_true'],integration=integration),ensure_ascii=False))


if __name__=='__main__': main(sys.argv[1])
