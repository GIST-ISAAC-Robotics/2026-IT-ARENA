"""계획 r361을 유지하고 현재 검사에 r375을 명시 연결한다."""
import importlib.util
import hashlib
import json
import os
from pathlib import Path
import time

A = Path(__file__).resolve().parent


def load_policy(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    policy = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(policy)
    return policy


def receipt_path(stage='plan'):
    # 노드 연결 시험은 tmp_path를 지정한다. 실주행 기본 경로는 이 시험 폴더다.
    folder = Path(os.environ.get('R377_POLICY_RECEIPT_DIR', str(A/'worker_policy_receipts')))
    folder.mkdir(parents=True, exist_ok=True)
    return folder/(stage+'_'+str(os.getpid())+'_'+str(time.time_ns()))


def write_receipt(prefix, suffix, record):
    with prefix.with_name(prefix.name+'_'+suffix+'.json').open('x', encoding='utf8') as stream:
        json.dump(record, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def worker(stage, *args):
    import worker_candidate as prior
    import route_proof
    old = route_proof.rebase
    boundary = prior.load_candidate()
    try:
        if stage == 'plan':
            boundary.install()
            source = A.parent/'released_crawl_r361_v1/candidate.py'
            policy = load_policy('r377_released_plan_policy', source)
            from arena_autonomy.pass_release_trial import PassReleaseSelector
            from arena_autonomy.bypass_handoff import BypassHandoffFollow
            original_select = PassReleaseSelector._select
            original_command = BypassHandoffFollow.command
            prefix = receipt_path()
            completed = False
            evidence = dict(stage='plan', pid=os.getpid(), policy='r361',
                candidate_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                boundary_policy_installed_first=True, default_changed=False,
                hardware_enabled=False, current_policy='r375_released_relative_body',
                scope='worker-local installation receipt; no command or drive acceptance claim')
            try:
                with policy.enabled():
                    assert PassReleaseSelector._select is not original_select
                    assert BypassHandoffFollow.command is not original_command
                    write_receipt(prefix, 'installed', {**evidence,
                        'selector_patch_installed': True, 'raw_speed_boundary_installed': True,
                        'selector_module': PassReleaseSelector._select.__module__,
                        'command_module': BypassHandoffFollow.command.__module__,
                        'before_original_worker': True})
                    prior.ORIGINAL(stage, *args)
                    completed = True
            finally:
                write_receipt(prefix, 'exit', {**evidence, 'worker_returned_normally': completed,
                    'selector_restored': PassReleaseSelector._select is original_select,
                    'raw_speed_boundary_restored': BypassHandoffFollow.command is original_command})
        elif stage == 'current':
            source = A.parent/'released_current_r375_v1/candidate.py'
            policy = load_policy('r375_current_worker', source)
            route_proof.rebase = policy.rebase
            prefix=receipt_path('current')
            evidence=dict(stage=stage,pid=os.getpid(),policy='r375',
                candidate_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                default_changed=False,hardware_enabled=False)
            completed=False
            try:
                write_receipt(prefix,'installed',evidence)
                prior.ORIGINAL(stage,*args)
                completed=True
            finally:
                write_receipt(prefix,'exit',{**evidence,'worker_returned_normally':completed})
        else:
            prior.ORIGINAL(stage, *args)
    finally:
        if stage == 'plan':
            boundary.uninstall()
        route_proof.rebase = old


def install():
    import overlap
    overlap._worker = worker
