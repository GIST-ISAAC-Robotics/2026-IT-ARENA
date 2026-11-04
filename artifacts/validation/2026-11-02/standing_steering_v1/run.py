"""상향 카메라 선두 출발·두 마커 이후 진행·실제 브라우저 STOP의 평가 전용 실행."""
import argparse
from hashlib import sha256
import importlib.util
import inspect
import json
from pathlib import Path
import sys
import time
from zipfile import ZipFile, ZIP_DEFLATED

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
PROFILE = HERE.parent/'official_camera_mount_v1/b0183_up20.yaml'
RUN = HERE/'run1'


def digest(path):
    return sha256(Path(path).read_bytes()).hexdigest()


def registration():
    parent = HERE.parent/'official_camera_mount_v1/run1_registered.json'
    files = set(json.loads(parent.read_bytes())['source_sha256'])
    files |= {str(p.relative_to(ROOT)).replace('\\', '/') for p in (
        PROFILE, Path(__file__), ROOT/'scripts/race_ops_eval.py',
        ROOT/'config/tests/standing_steering_r1_sha256.json')}
    for directory in ('src/arena_autonomy/arena_autonomy', 'src/arena_vehicle_interface/arena_vehicle_interface',
                      'src/arena_bringup/launch'):
        files |= {p.relative_to(ROOT).as_posix() for p in (ROOT/directory).glob('*.py')}
    return dict(scope='시뮬 장착 후보. 선두 index2·상향20도·1.5m/s 구성, s38m 이후 실제 화면STOP. 정답은 평가 전용.',
                grid_slot=2, route_target_s_m=38., speed_profile='g1_open_1p5',
                local_sensor_config=PROFILE.relative_to(ROOT).as_posix(),
                source_sha256={p:digest(ROOT/p) for p in sorted(files)})


def until_course(self, target, timeout, label):
    deadline=time.monotonic()+timeout*self.scale
    while time.monotonic()<deadline:
        latest=self.truth.latest()
        progress,_=self.course.nearest(latest[1],latest[2])
        if target<=progress<target+6.: return progress
        if (self.ops.state or {}).get('race',{}).get('latch'):
            raise EVAL.Stop('operator STOP before route target: '+label)
        time.sleep(.02)
    raise EVAL.Stop('truth progress timeout: '+label)


def mounted_course(self):
    self.ops.wait(lambda s: s['binding'].get('session') and s['race'].get('latch') == 'process_start',
                  120., 'browser initial state')
    self.note('await_browser_prepare', camera_pitch_deg=-20., grid_slot=2)
    try:
        self.ops.wait(lambda s: s['race'].get('started') is True, 240., 'browser prepare and RGB signal')
        source = self.ops.state['race']['start_source']
        if source != 'visual':
            raise EVAL.Stop('first start is not visual: '+str(source))
        self.note('browser_visual_started', source=source)
        reached = self.until_s(38., 90., 'camera route past both markers')
        ids = {int(i) for row in self.ops.history for i in (row['driving'].get('marker_ids') or [])}
        self.note('camera_route_reached', progress_s_m=reached, observed_marker_ids=sorted(ids))
        # 마커 확인이 실패해도 먼저 화면 STOP·실제 정지 증거를 회수한다.
        self.note('await_browser_stop')
        self.ops.wait(lambda s: bool(s['race'].get('latch')) and s['mcu']['state']=='ESTOP_LATCHED',
                      180., 'actual browser STOP')
        self.stopped()
        self.hold(.3)
        self.note('browser_stop_observed', truth_speed=self.truth.speed(), ui=self.ops.state['state'],
                  mcu=self.ops.state['mcu']['state'], commands=self.ops.state['commands'])
        self.note('await_browser_snapshot')
        deadline = time.monotonic()+90.
        while not (self.output/'browser_capture_done').exists() and time.monotonic()<deadline:
            time.sleep(.1)
        if not {20,30} <= ids:
            raise EVAL.Stop('course marker coverage incomplete: '+str(sorted(ids)))
    finally:
        self.stop_car()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    if RUN.exists():
        raise FileExistsError('existing run1; automatic retries are not permitted')
    registered = registration()
    path = HERE/'run1_registered.json'
    if args.prepare_only:
        with path.open('x', encoding='utf-8') as out:
            json.dump(registered,out,ensure_ascii=False,indent=2)
        with ZipFile(HERE/'run1_source_snapshot.zip','x',ZIP_DEFLATED) as archive:
            for name in registered['source_sha256']:
                archive.write(ROOT/name,name)
        print('registered',len(registered['source_sha256']),'files')
        raise SystemExit(0)
    if registered != json.loads(path.read_bytes()):
        raise ValueError('registered source changed')
    import validate_basic_autonomy as basic
    original_start = basic.start_demo_process
    signature = inspect.signature(original_start)

    def start_with_camera(*args, **kwargs):
        bound = signature.bind(*args, **kwargs)
        bound.apply_defaults()
        if bound.arguments['speed_profile'] != 'g1_open_1p5':
            raise ValueError('unexpected speed profile')
        bound.arguments['grid_slot'] = 2
        bound.arguments['extra_arguments'] = [*bound.arguments['extra_arguments'], f'local_sensor_config:={PROFILE}']
        return original_start(*bound.args, **bound.kwargs)

    basic.start_demo_process = start_with_camera
    spec = importlib.util.spec_from_file_location('mounted_camera_evaluation',ROOT/'scripts/race_ops_eval.py')
    EVAL = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(EVAL)
    EVAL.Scenario.browser_only = mounted_course
    EVAL.Scenario.until_s = until_course
    sys.argv = ['race_ops_eval.py','--output',str(RUN),'--scenario','browser_stop',
                '--wall-limit-s','1800','--wall-scale','14']
    code = EVAL.main()
    mismatches = [p for p,h in registered['source_sha256'].items() if digest(ROOT/p)!=h]
    result = dict(runtime_exit=code,source_mismatches=mismatches,
                  scope='별도 노면/경로/실제 화면STOP 사후 판정 필요. raw passed 초기값과 구별한다.')
    applied_path = RUN/'runtime_local_sensors.yaml'
    applied = yaml.safe_load(applied_path.read_text(encoding='utf-8')) if applied_path.exists() else None
    result['applied_profile_matches'] = applied == yaml.safe_load(PROFILE.read_text(encoding='utf-8'))
    (HERE/'run1_source_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    raise SystemExit(code if code else (1 if mismatches or not result['applied_profile_matches'] else 0))
