"""JR60P20·Ø20 배선 단독 1랩 기준 주행의 사전 등록. 실행 전에 한 번만 만든다(WSL python3 -I -B).

입력: input_list.txt(Windows git ls-files -co --exclude-standard src scripts config, 실행 전 저장).
출력: registered.json, registered_inputs.sha256. 기존 등록이 있으면 덮어쓰지 않는다.
"""
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
REL = HERE.relative_to(ROOT).as_posix()
RUN = f'{REL}/run1'
ENV = 'artifacts/validation/2026-11-01/turn_preview_v1/environment.sh'
EXTRA = [ENV, 'artifacts/validation/2026-11-01/race_continuity_v1/measure_solo.py',
         f'{REL}/register.py', f'{REL}/verify.py', f'{REL}/run.sh', f'{REL}/post.sh', f'{REL}/analyze.py',
         f'{REL}/input_list.txt', f'{REL}/.gitignore']
# 오늘 같은 장착·월드의 OFF 기준 run1(32.949196 s). 1회끼리의 비교이며 반복 분산은 모른다.
PRIOR = 'artifacts/validation/2026-11-05/solo_jr60p20_baseline_v1'


def sha(path):
    return hashlib.sha256((ROOT/path).read_bytes()).hexdigest()


def main():
    out, sums = HERE/'registered.json', HERE/'registered_inputs.sha256'
    if out.exists() or sums.exists():
        sys.exit('registration already exists; not overwritten')
    listed = [line.strip() for line in (HERE/'input_list.txt').read_text(encoding='utf-8').splitlines() if line.strip()]
    files = sorted(set(listed) | set(EXTRA))
    missing = [p for p in files if not (ROOT/p).is_file()]
    if missing:
        sys.exit(f'missing inputs: {missing}')
    digests = {p: sha(p) for p in files}
    args = json.loads(subprocess.run([sys.executable, '-I', '-B', 'scripts/run_current_solo.py', RUN, '--print-args', '--straight-budget'],
                                     cwd=ROOT, check=True, capture_output=True, text=True).stdout)
    vehicle = yaml.safe_load((ROOT/'src/arena_description/config/vehicle.yaml').read_text(encoding='utf-8'))['vehicle']
    sensor_path = 'src/arena_description/config/b0183_c1_jr60p20_sim.yaml'
    sensor = yaml.safe_load((ROOT/sensor_path).read_text(encoding='utf-8'))
    drive = vehicle['sensors']['drive_feedback']
    links = {}
    for link in ['install/arena_autonomy/lib/python3.12/site-packages/arena-autonomy.egg-link',
                 'install/arena_vehicle_interface/lib/python3.12/site-packages/arena-vehicle-interface.egg-link']:
        p = ROOT/link
        links[link] = p.read_text(encoding='utf-8').splitlines()[0] if p.exists() else None
    for link in ['build/arena_autonomy/arena_autonomy', 'build/arena_vehicle_interface/arena_vehicle_interface',
                 'install/arena_bringup/share/arena_bringup/launch/simulation.launch.py',
                 'install/arena_gazebo/share/arena_gazebo/worlds/it_arena_official/world.sdf',
                 'install/arena_gazebo/share/arena_gazebo/worlds/it_arena_official/scene.json',
                 'install/arena_description/share/arena_description/config/vehicle.yaml',
                 'install/arena_gazebo/lib/libarena-single-motor-system.so']:
        p = ROOT/link
        links[link] = os.readlink(p) if p.is_symlink() else ('regular' if p.exists() else None)
    plugin = ROOT/'build/arena_gazebo/libarena-single-motor-system.so'
    prior_metrics = json.loads((ROOT/PRIOR/'run1_metrics.json').read_bytes())
    prior_eval = json.loads((ROOT/PRIOR/'run1_analysis.json').read_bytes())
    registered = dict(
        schema='solo_straight_closed_loop_v1', registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        purpose='단독 직선 속도 후보(solo_straight_budget, r15 계약, 기본 OFF를 --straight-budget로 ON)의 Gazebo 단독 1랩 1회. 나머지 조건은 오늘 run1 기준과 같다',
        rules=['1회만 실행한다. 실패해도 같은 주행을 반복하지 않는다.',
               '등록 뒤 추월·제어기·공식 월드·런타임 코드와 판정선을 바꾸지 않는다. 후보 A·녹화 카메라 변경은 포함하지 않는다.',
               '과거 소스 묶음·staged install을 쓰지 않는다. 진입점은 scripts/run_current_solo.py 하나다.',
               '평가 정답(자세·신호 상태)은 사후 집계에만 쓰며 제어 입력이 아니다.',
               '오늘 run1(OFF) 32.949196 s·지면 최고 2.3930185 m/s와 비교한다. 1회끼리 비교라 반복 분산은 모른다.'],
        entry=dict(command=f'bash {ENV} /usr/bin/python3 -B scripts/run_current_solo.py {RUN} --straight-budget', print_args=args,
                   output=RUN, environment=ENV),
        fixed_conditions=dict(
            sensor_config=sensor_path, lidar_xyz_m=sensor['lidar']['xyz_m'], scan_plane_posts=sensor['lidar']['scan_plane_posts'],
            scan_plane_wires=sensor['lidar']['scan_plane_wires'], known_structure_mask=sensor['lidar']['known_structure_mask'],
            lidar_rate_hz=sensor['lidar']['rate_hz'], pilot_marker_z_m=float(args[args.index('--pilot-marker-z')+1]),
            drive_feedback_mode=drive['mode'], counts_per_motor_revolution=drive['drive_motor_encoder']['counts_per_motor_revolution'],
            gear_ratio=vehicle['drivetrain']['motor']['gear_ratio'], wheel_radius_m=vehicle['drivetrain']['wheel_radius_m'],
            c1_range_max_expected_m=12.0, render_backend=args[args.index('--render-backend')+1],
            speed_profile=args[args.index('--speed-profile')+1], speed_bump='disabled (--disable-speed-bump)',
            world='src/arena_gazebo/worlds/it_arena_official (v2026.11.01 team derivative, tl_post removed)',
            protection='/drive → lidar_safety → /drive/safe (launch default for local_pursuit; checked after run from report)'),
        install_links=links,
        gazebo_plugin=dict(path='build/arena_gazebo/libarena-single-motor-system.so', sha256=sha('build/arena_gazebo/libarena-single-motor-system.so'),
                           mtime_utc=datetime.datetime.fromtimestamp(plugin.stat().st_mtime, datetime.timezone.utc).isoformat(),
                           note='C++ 플러그인은 빌드 산출물이다. src/arena_gazebo 작업트리 변경 없음을 git_state_registered.txt로 확인.'),
        measurement=dict(
            target_mps=10/3.6, target95_mps=.95*10/3.6, sustained_threshold_mps=2.0,
            start='실제 RGB 출발: 자율 상태 signal=green_confirmed·started 첫 표본, 검사기 official_signal_evidence(start_after_actual_green), 첫 0.1 m/s 이동',
            finish='measure_solo.py lap_s_from_first_green_sample와 report lap_length_m·progress_m',
            speed='평가 정답 지면속도 hypot(종,횡) 선형보간. 최고·최고 1초 평균·임계 이상 총/최장 연속 시간',
            zero_command='follow_decisions 출력 속도 ≤0.001 호출 × 0.02 s, 사유·진행 위치별 구간; 출발 전/후 분리',
            loss_42_44='progress_m 42–44 m 통과 시간·평균/최저 속도·그 구간 0명령',
            safety='/drive/safe가 /drive보다 0.05 m/s 이상 낮은 표본, safety.reason 시간',
            surface='scripts/surface_contact_eval.py', motor='scripts/audit_motor_encoder_runtime.py',
            stop_shutdown='report stop_stable·dense_pose_stop_evidence·actuation final_mcu·shutdown_audit·remaining_test_pids',
            missing='표본 최대 간격·라이다 주기·센서/명령 수신·영상 선언/디코드 프레임',
            video='report.video 존재·프레임·시뮬 시각만 확인. 별도 촬영·재시간화 없음'),
        prior_reference=dict(
            note='오늘 OFF 기준 run1. 같은 장착·월드, 후보 OFF.',
            run=f'{PRIOR}/run1', lap_s=prior_metrics['lap_s_from_first_green_sample'],
            peak_ground_mps=prior_metrics['peak_ground_mps'], best_one_second_mean_mps=prior_metrics['best_one_second_mean_mps'],
            time_above_95_percent_s=prior_metrics['time_above_95_percent_s'],
            zero_command_s=prior_eval['now']['zero_command']['zero_command_s'], zero_by_reason=prior_eval['now']['zero_command']['by_reason'],
            loss_42_44_m=prior_eval['now']['loss_42_44_m']['traverse_s']),
        input_count=len(digests), input_sha256=digests)
    sums.write_text(''.join(f'{d}  {p}\n' for p, d in digests.items()), encoding='utf-8', newline='\n')
    out.write_text(json.dumps(registered, ensure_ascii=False, indent=1), encoding='utf-8', newline='\n')
    print(json.dumps(dict(inputs=len(digests), args=len(args), out=str(out.relative_to(ROOT))), ensure_ascii=False))


if __name__ == '__main__':
    main()
