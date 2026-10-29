import argparse
import json
import sys
from pathlib import Path
import pytest
from trial_contract import route_contract,validate_result


@pytest.mark.parametrize('laps',[1,2,5])
def test_course_arguments_override_evaluator_default(laps):
    contract=route_contract('course',laps,46.6329)
    parser=argparse.ArgumentParser()
    parser.add_argument('--laps',type=int,default=2)
    parser.add_argument('--target-progress-m',type=float)
    result=parser.parse_args(contract['arguments'])
    assert result.laps==laps
    assert result.target_progress_m==pytest.approx(laps*46.6329+2)
    assert contract['full_lap_validation']


def test_crossing_has_an_explicit_short_distance():
    c=route_contract('crossing',1,46.6329)
    assert c['target_progress_m']==6.2 and not c['full_lap_validation']


@pytest.mark.parametrize('scope,laps',[('unknown',1),('course',0),('course',6),
                                     ('course',True),('crossing',2)])
def test_unintended_route_is_rejected(scope,laps):
    with pytest.raises(ValueError):route_contract(scope,laps,46.6329)


def test_wrong_report_laps_or_distance_cannot_pass_contract():
    c=route_contract('course',1,46.6329)
    report={key:c[key] for key in ('requested_laps','lap_length_m','target_progress_m','full_lap_validation')}
    assert validate_result(c,report)['passed']
    for key,value in [('requested_laps',2),('target_progress_m',95.2658),
                      ('lap_length_m',None),('full_lap_validation',False)]:
        assert not validate_result(c,{**report,key:value})['passed']


@pytest.mark.parametrize('scope,laps',[('crossing',1),('course',1),('course',2)])
def test_main_passes_prepared_route_to_evaluator(tmp_path,monkeypatch,scope,laps):
    from test_adapter import runner
    base=tmp_path/'trial';base.mkdir();fixture=tmp_path/'fixture';fixture.mkdir()
    (fixture/'scene.json').write_text(json.dumps({'track':{'lap_length_m':46.6329}}))
    (tmp_path/'scripts').mkdir();(tmp_path/'scripts/validate_basic_autonomy.py').write_text('pass\n')
    out=base/'new_run';contract=route_contract(scope,laps,46.6329)
    metadata=dict(output=str(out),launch_arguments={'speed_profile':'brisk'},trial_scope=scope,
        route_contract=contract,source_sha256={},launch_path=str(base/'dummy.launch.py'))
    (base/'new_run_prepared.json').write_text(json.dumps(metadata))
    monkeypatch.setattr(runner,'ROOT',tmp_path);monkeypatch.setattr(runner,'BASE',base)
    monkeypatch.setattr(runner,'FIXTURE',fixture)
    # 실제 main 경유 인수만 받아 보고서를 쓰며 ROS/Gazebo는 시작하지 않는다.
    evaluator='''
import argparse,json,sys
from pathlib import Path
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--laps',type=int,default=2)
    parser.add_argument('--target-progress-m',type=float)
    parser.add_argument('--output',type=Path)
    args,_=parser.parse_known_args()
    args.output.mkdir()
    report=dict(requested_laps=args.laps,lap_length_m=46.6329,
        target_progress_m=args.target_progress_m,full_lap_validation=args.target_progress_m>=46.6329)
    (args.output/'report.json').write_text(json.dumps(report))
    return 0
'''
    monkeypatch.setattr(runner,'patch_evaluator',lambda _:evaluator)
    monkeypatch.setattr(sys,'argv',['run.py','--output',str(out),'--scope',scope,'--laps',str(laps),'--speed-profile','brisk'])
    monkeypatch.setattr(sys,'path',list(sys.path))
    assert runner.main()==0
    overlay=json.loads((out/'trial_overlay.json').read_text())
    assert overlay['route_contract_check']['passed']
    assert overlay['argv'][overlay['argv'].index('--laps')+1]==str(laps)
