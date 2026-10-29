"""기존 오른쪽 G2 배치/노면은 유지하고 실행 파일·G1/G2 계약만 사전 등록한다."""
from dataclasses import asdict
import difflib
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
OLD=BASE.parent/'g2_entry_gazebo_v1';LEFT=BASE.parent/'g2_entry_left_v1'
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from arena_autonomy.bypass_handoff import HandoffExecutionConfig


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path,value):
    with path.open('x',encoding='utf-8',newline='\n') as stream:stream.write(value)


def main():
    fixture=BASE/'fixture';fixture.mkdir(exist_ok=False)
    original=(OLD/'fixture/simulation.launch.py').read_text()
    assert original.count('"g2_entry_trial"')==1
    changed=original.replace('"g2_entry_trial"','"g1_g2_handoff_trial"')
    write(fixture/'simulation.launch.py',changed)
    write(fixture/'simulation.diff',''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True),
        fromfile='g2_entry_gazebo_v1/fixture/simulation.launch.py',tofile='handoff/fixture/simulation.launch.py')))
    manifest=json.loads((OLD/'fixture/manifest.json').read_text())
    assert all(sha(ROOT/p)==h for p,h in manifest['sources'].items())
    manifest['scope']='G1/G2 handoff executable; unchanged right-edge placement and official signal v2 comparison world'
    dependencies=[Path(__file__),OLD/'fixture/simulation.launch.py',OLD/'preflight.py',LEFT/'g2_config_v2_left.json']
    manifest['sources'].update({p.relative_to(ROOT).as_posix():sha(p) for p in dependencies})
    write(fixture/'manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    config=json.loads((LEFT/'g2_config_v2_left.json').read_text())
    case=config['cases'][0];case['name']='handoff_edge_stopped_right';case['actor_lateral_offset_m']=-.149
    case['expected']='G1 follow -> observed low-motion candidate -> G2 entry -> G1 reacquire -> explicit STOP'
    config['acceptance'][case['name']]=config['acceptance'].pop('g2_edge_stopped_left')
    config['matrix']['g2']['cases']=[case['name']]
    config['scope']='Explicit G1/G2 handoff; one right-edge stationary box, signal v2 comparison world, bump OFF; not full-lap or hardware proof'
    config['registered_at']='2026-10-29 before first G1/G2 handoff Gazebo run'
    rules=config['acceptance']['g2'];rules['version']='handoff_acceptance_v1'
    rules['command_cap_mps']=1.
    rules['command_cap_scope']='FOLLOW <=1 m/s; BYPASS/REACQUIRE <=0.35 m/s checked separately'
    rules['decision_replay']='schema5 in g2_decisions.jsonl; filename retained for evaluator compatibility'
    rules['trace_header_basis']='registered integrated G1 plus explicit handoff; not historical G2 defaults'
    head=rules['trace_header_expected'];head['schema']=5;head['controller_mode']='g1_g2_handoff_trial'
    head['execution_config']=asdict(HandoffExecutionConfig())
    follow=head['follow_config'];follow.update(partial_observation_policy='bounded_local_v4',
        boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate')
    for key in ('parallel_hold_trial','pass_release_trial','guard_curve_trial','traffic_recovery_trial',
                'traffic_path_recovery_trial','corridor_fallback_enabled'):follow[key]=True
    write(BASE/'g2_config_v1.json',json.dumps(config,ensure_ascii=False,indent=2)+'\n')
    spec=importlib.util.spec_from_file_location('handoff_layout_preflight',OLD/'preflight.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.BASE=BASE
    code=module.main()
    if code:raise SystemExit(code)


if __name__=='__main__':main()
