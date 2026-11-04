"""화면 없이 같은 HTTP 경로로 일반 STOP→START를 1회 확인하는 평가 전용 실행."""
import argparse
import importlib.util
import inspect
import json
from pathlib import Path
import sys
import time
from zipfile import ZipFile, ZIP_DEFLATED

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RUN = HERE/'run1'
sys.path.insert(0, str(ROOT/'scripts'))
from g1_traffic_eval import stop_pose_evidence


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BASE = load('post_free_registered_evaluation', HERE.parent/'post_free_facility_v1/run.py')
EVAL = load('ordinary_resume_evaluation', ROOT/'scripts/race_ops_eval.py')


def stopped_continuously(self):
    deadline = time.monotonic()+15*self.scale
    while time.monotonic() < deadline:
        rows = [r[:4] for r in self.truth.rows[-300:]]
        if rows and stop_pose_evidence(rows, rows[-1][0], .2)['passed']:
            return
        time.sleep(.05)
    raise EVAL.Stop('continuous 0.2s stop not observed')


def ordinary_course(self):
    self.first_start()
    initial_binding = dict(self.ops.state['binding'])
    self.wait(lambda s: (s['speeds']['protected_mps'] or 0.) > .25, 5., 'moving before STOP')
    self.note('moving_before_stop', body_speed_mps=self.truth.speed())
    try:
        self.stop_car()
        stopped = self.truth.latest()
        self.note('ordinary_stopped', stop_evidence=stop_pose_evidence([r[:4] for r in self.truth.rows[-300:]], stopped[0], .2))
        self.wait(lambda s: s['controls']['resume_start']['allowed'], 3., 'START without RESET')
        if self.ops.state['race']['recovery_ready']:
            raise EVAL.Stop('RESET unexpectedly occurred')
        entry = self.press('resume_start')
        # 완료 단계는 수락보다 먼저 관측될 수 있다. 기존 resume()가 준비/완료를 모두 확인한다.
        self.resume('ordinary_no_reset', entry)
        begin_s, _ = self.course.nearest(stopped[1], stopped[2])
        reached = self.until_s(begin_s+3., 20., 'three metres after ordinary resume')
        binding = self.ops.state['binding']
        same_race = all(binding[k] == initial_binding[k] for k in ('session', 'race_id', 'race_epoch'))
        if not same_race:
            raise EVAL.Stop('ordinary resume changed race')
        if any(s['step'] == 'press_recovery_reset' for s in self.steps):
            raise EVAL.Stop('unexpected RESET command')
        self.note('ordinary_progress', start_s_m=begin_s, end_s_m=reached,
                  progress_m=reached-begin_s, same_race=same_race)
    finally:
        self.stop_car()
        self.hold(.2)
        latest = self.truth.latest()
        self.note('final_continuous_stop', evidence=stop_pose_evidence([r[:4] for r in self.truth.rows[-300:]], latest[0], .2))


def main():
    global RUN
    parser = argparse.ArgumentParser()
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--attempt', choices=('run1', 'run2'), default='run1')
    args = parser.parse_args()
    RUN = HERE/args.attempt
    if RUN.exists():
        raise FileExistsError('run1 already exists; no automatic retry')
    reg = BASE.registration()
    reg.update(scope='기둥 없는 선두index2·상향20도·1.5m/s 후보, 자동 HTTP 최초 준비/시각 출발→일반 STOP→RESET 없는 START→3m 진행→STOP. 정답은 평가 전용.',
               route_target_s_m=None, resume_progress_min_m=3., stop_max_speed_mps=.01, stop_hold_s=.2)
    for p in (Path(__file__), ROOT/'config/tests/ordinary_resume_r1_sha256.json'):
        reg['source_sha256'][p.relative_to(ROOT).as_posix()] = BASE.digest(p)
    path = HERE/f'{args.attempt}_registered.json'
    if args.prepare_only:
        with path.open('x', encoding='utf-8') as f:
            json.dump(reg, f, ensure_ascii=False, indent=2)
        with ZipFile(HERE/f'{args.attempt}_source_snapshot.zip', 'x', ZIP_DEFLATED) as z:
            for p in reg['source_sha256']:
                z.write(ROOT/p, p)
        print('registered', len(reg['source_sha256']), 'files')
        return 0
    if reg != json.loads(path.read_bytes()):
        raise ValueError('registered source changed')
    import validate_basic_autonomy as basic
    original = basic.start_demo_process
    signature = inspect.signature(original)

    def start(*args, **kwargs):
        bound = signature.bind(*args, **kwargs)
        bound.apply_defaults()
        assert bound.arguments['speed_profile'] == 'g1_open_1p5'
        bound.arguments['grid_slot'] = 2
        bound.arguments['extra_arguments'] = [*bound.arguments['extra_arguments'], f'local_sensor_config:={BASE.PROFILE}']
        return original(*bound.args, **bound.kwargs)

    basic.start_demo_process = start
    EVAL.Scenario.browser_only = ordinary_course
    EVAL.Scenario.stopped = stopped_continuously
    sys.argv = ['race_ops_eval.py', '--output', str(RUN), '--scenario', 'browser_stop',
                '--first-stop-s', '6', '--wall-limit-s', '900', '--wall-scale', '14']
    code = EVAL.main()
    mismatches = [p for p,h in reg['source_sha256'].items() if BASE.digest(ROOT/p) != h]
    result = dict(runtime_exit=code, source_mismatches=mismatches,
                  scope='브라우저 표시 없이 동일 HTTP 명령 경로. 원본 report.passed 초기값과 실제 종료/사후 판정 구별.')
    (HERE/f'{args.attempt}_source_audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    return code or int(bool(mismatches))


if __name__ == '__main__':
    raise SystemExit(main())
