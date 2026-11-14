"""r500 설치본을 바꾸지 않는 20초 ROS 재구성 재생 입력 준비."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
B = A / 'bundle'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    B.mkdir(exist_ok=False)
    source = A.parent / 'jetson_lap_cost_r493_v1/bundle/inputs.json'
    assert sha(source) == '909c107fe700cf751e0a8c09b154bec09bff99140cd78b6b449014f8dfa829ac'
    original = json.loads(source.read_bytes())
    start = original['plans'][0]['input']['reference_s']
    finish = start + 20.
    plans = [p for p in original['plans'] if start <= p['input']['reference_s'] < finish]
    events, histories = [], {'wheels': {}, 'gyro': {}}
    for plan in plans:
        call = plan['input']
        scan = dict(call['scan']['measurements'], schema='arena_native_lidar_v1')
        assert scan['time_basis'] == 'simulation_acquisition'
        scan['ranges_m'] = [({'nan': None, 'inf': '+inf', '-inf': '-inf'}[v['nonfinite']]
                             if isinstance(v, dict) else v) for v in scan['ranges_m']]
        events.append(dict(at=call['reference_s'], kind='scan', value=scan, sequence=plan['sequence']))
        for kind, series in call['history'].items():
            for t, v in zip(series['times'], series['values']):
                key = round(t * 1e9)
                if key in histories[kind]:
                    assert histories[kind][key] == (t, v)
                histories[kind][key] = (t, v)
    for kind, series in histories.items():
        events.extend(dict(at=t, kind=kind, value=v) for t, v in series.values())
    events.sort(key=lambda row: (row['at'], row['kind']))
    data = dict(header=original['header'], first_reference=start, last_reference=finish,
                first_time=events[0]['at'], events=events)
    with gzip.open(B / 'reconstructed_inputs.json.gz', 'wt', encoding='utf8') as stream:
        json.dump(data, stream, separators=(',', ':'), allow_nan=False)
    # This old explicit profile is only a parameter mapping template. All clock,
    # frame and runtime choices below are selected for simulation, not hardware.
    template = ROOT / 'artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml'
    profiles, active = {}, None
    for line in template.read_text(encoding='utf8').splitlines():
        if line and not line.startswith((' ', '#')):
            active = line[:-1]
            profiles[active] = {'ros__parameters': {}}
        elif line.startswith('    '):
            key, value = line.strip().split(': ', 1)
            profiles[active]['ros__parameters'][key] = json.loads(value)
    for name, body in profiles.items():
        params = body['ros__parameters']
        params.update(execution_environment='default', use_sim_time=True,
            lidar_input_representation='native_simulation_v1', motion_scan_timing_verified=True,
            motion_scan_time_policy='verified_only', motion_imu_topic='/imu/data',
            motion_imu_frame_id='r506_reconstructed_sim_imu', motion_lidar_frame_id='laser_frame',
            g1_guard_observation_policy='selected_r181')
    profiles['local_pursuit']['ros__parameters'].update(
        g1_current_corner_budget='r396_v1', g1_crop_edge_component_trial=True,
        g1_complete_plan_trace=False, race_control_mode='legacy', timing_probe=False)
    (B / 'profile.json').write_text(json.dumps(profiles, indent=2) + '\n', encoding='utf8')
    for name in ('node_runner.py', 'run_replay.py', 'analyze.py'):
        shutil.copyfile(A / name, B / name)
    manifest = dict(schema=1, harness_revision=4,
        observer='dedicated executor thread; prebuilt RGB array; no callback queue sampled by publication loop',
        input_source=str(source.relative_to(ROOT)).replace('\\', '/'),
        input_source_sha256=sha(source), root_head='b59efcab11df06a55d85620ef24433af9b8fd521',
        r500_source_zip_sha256='73bebdcdc176eef04b641dc78c986b76ec32431b194c787cba0f351ad6f95965',
        r500_external_manifest_sha256='ee450fef0d971d2b94e75544420de5e13d6e9e694b4d2a12bdf6f848581098a5',
        r500_embedded_manifest_sha256='64062a452947884c4269f36f3158aec3d57aa9bcb7c2222b25a45c45abf73cbc',
        installed_root='/home/q/it-arena-field-r500-20261114-a', namespace='/replay_r506', ros_domain=156,
        representative_seconds=20., rate=1., first_reference=start, last_reference=finish,
        motion_prefix_seconds=start-events[0]['at'], eof_tail_seconds=1., hard_deadline_seconds=60.,
        event_counts={kind: sum(e['kind'] == kind for e in events) for kind in ('scan', 'gyro', 'wheels')},
        raw_cdr_replay=False, timestamps_rebased=False, native_time_basis='simulation_acquisition',
        scan_publication_time='stored evaluator reference_s; original DDS arrival timing unavailable',
        encoder_reconstruction='stored mean linear velocity inverted to motor angular velocity; zero position placeholder only',
        rgb='existing light_image synthetic 848x480 pixels at20Hz off/red1..4/green; actual detector; no B0390 inference',
        runtime='installed G1G2 native2slot simulation backend + independent guard + read-only RaceOps',
        absent=['physical ports', 'runtime host', 'MCU', 'B3 start', 'real RGB', 'Gazebo', 'closed-loop response'],
        profile_template_sha256=sha(template), physical_output=False,
        files={p.name: sha(p) for p in B.iterdir() if p.is_file()})
    (B / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf8')
    shutil.copyfile(B / 'manifest.json', A / 'manifest.json')
    with zipfile.ZipFile(A / 'bundle.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in B.iterdir():
            archive.write(path, path.name)
    print(json.dumps(dict(counts=manifest['event_counts'], prefix=manifest['motion_prefix_seconds'],
        bundle_sha256=sha(A / 'bundle.zip'))))


if __name__ == '__main__':
    main()
