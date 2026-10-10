"""평가 전용 예약 적용 r7 사전 등록/소스 사본 생성. 기존 파일을 덮어쓰지 않는다."""
import hashlib
import json
import zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
BASE=Path(__file__).resolve().parent
PARENT='config/tests/scheduled_command_r6_sha256.json'
sha=lambda p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
old=json.loads((ROOT/PARENT).read_text())
before=json.loads((BASE/'before_manifest.json').read_text())
groups={
    'approved_changes':['src/arena_bringup/launch/simulation.launch.py'],
    'approved_evaluation_scripts':['scripts/probe_braking_contact.py'],
    'approved_implementation_changes':['src/arena_gazebo/src/single_motor_drive.cpp',
        'scripts/evaluation_startup.py','src/arena_gazebo/include/arena_gazebo/scheduled_apply.hpp',
        'src/arena_vehicle_interface/arena_vehicle_interface/evaluation_schedule.py',
        'src/arena_vehicle_interface/arena_vehicle_interface/scheduled_actuator.py',
        'src/arena_vehicle_interface/setup.py']}
c={**old,'scope':'2026-10-11 평가 전용 paired 20 ms scheduled application / receipt and apply ACK',
   'parent_contract':PARENT,'parent_contract_sha256':sha(PARENT)}
for group,paths in groups.items():
    c[group]={p:{'before':before.get(p),'after':sha(p),'reason':'평가 전용 예약 적용; 기본 경로와 보호 계약 유지'} for p in paths}
for p in groups['approved_changes']+groups['approved_evaluation_scripts']:c['sha256'][p]=sha(p)
for p in groups['approved_implementation_changes']:c['implementation_sha256'][p]=sha(p)
with (ROOT/'config/tests/scheduled_command_r7_sha256.json').open('x') as f:json.dump(c,f,ensure_ascii=False,indent=2)
files=set(c['sha256'])|set(c['implementation_sha256'])|set(c['evaluation_tools_sha256'])
files.update(before)
with zipfile.ZipFile(BASE/'registered_source.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(files):z.write(ROOT/p,p)
with (BASE/'registered_manifest.json').open('x') as f:
    json.dump({'sources':{p:sha(p) for p in sorted(files)},
               'plugin_sha256':sha('install/arena_gazebo/lib/libarena-single-motor-system.so'),
               'contract_sha256':sha('config/tests/scheduled_command_r7_sha256.json')},f,indent=2)
profile='artifacts/validation/2026-10-08/braking_ab_v1/command_profile.json'
args=['--surface','official','--zero-speed-mode','passive_damping','--command-schedule','sim_grid',
      '--schedule-phase-ms','10','--command-profile',profile,'--plugin-command-trace',
      '--evaluation-gc','defer','--evaluation-startup','embedded_paused']
for case in ('n1_native','t1_scheduled','t2_scheduled'):
    caseargs=args+(['--evaluation-scheduled-apply'] if case.startswith('t') else [])
    with (BASE/f'plan_{case}.json').open('x') as f:
        json.dump({'base':str(BASE.relative_to(ROOT)),'cases':[{'name':case,'expect':'pass','args':caseargs}]},f,indent=2)
print('registered r7; immutable sources, binary hash and three plans saved')
