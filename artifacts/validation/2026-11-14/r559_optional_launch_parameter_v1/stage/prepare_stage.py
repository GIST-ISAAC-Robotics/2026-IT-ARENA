"""r555 원본/로그를 읽고 최소 launcher 후보만 별도 저장한다."""
import ast
import difflib
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[4]
SOURCE = ROOT / 'artifacts/validation/2026-11-14/r555_sparse_actor_ready_v1/marker_course.launch.py'
LOG = SOURCE.parent / 'run1/simulation.log'
sha = lambda raw: hashlib.sha256(raw).hexdigest()
before = SOURCE.read_bytes()
text = before.decode('utf-8')
changes = [
    ('from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription',
     'from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction'),
    ('def generate_launch_description():',
     "def _marker_profile_parameters(context):\n"
     "    # 빈 global parameter는 ROS CLI name:=로 직렬화되어 init 단계에서 거절된다.\n"
     "    profile=LaunchConfiguration('r537_marker_profile').perform(context)\n"
     "    if not profile:return []  # 소비자의 기존 빈 경로 기본값: 마커 보조 OFF\n"
     "    return [SetParameter(name='vehicle_marker_profile_path',\n"
     "                         value=ParameterValue(profile,value_type=str))]\n\n\n"
     'def generate_launch_description():'),
    ("        SetParameter(name='vehicle_marker_profile_path',\n"
     "                     value=ParameterValue(LaunchConfiguration('r537_marker_profile'),value_type=str)),",
     '        OpaqueFunction(function=_marker_profile_parameters),'),
]
for old, new in changes:
    assert text.count(old) == 1, old
    text = text.replace(old, new)
ast.parse(text)
after = text.encode('utf-8')
for name, raw in [('marker_course.launch.py.before', before), ('marker_course.launch.py', after)]:
    with (A / name).open('xb') as stream:
        stream.write(raw)
patch = ''.join(difflib.unified_diff(before.decode('utf-8').splitlines(True), text.splitlines(True),
    fromfile='a/marker_course.launch.py', tofile='b/marker_course.launch.py'))
(A / 'optional_marker_parameter.patch').write_text(patch, encoding='utf-8', newline='\n')
lines = LOG.read_text(encoding='utf-8').splitlines()
error = next((i+1, line) for i,line in enumerate(lines) if "Couldn't parse parameter override rule: '-p vehicle_marker_profile_path:='" in line)
command = next((i+1, line) for i,line in enumerate(lines) if 'process has died' in line and ' -p vehicle_marker_profile_path:=' in line)
evidence = dict(source_log=LOG.relative_to(ROOT).as_posix(),
    original_error=dict(line=error[0], text=error[1]), actual_command=dict(line=command[0], text=command[1]),
    exact_failing_override='vehicle_marker_profile_path:=',
    original_source=SOURCE.relative_to(ROOT).as_posix(),
    original_source_sha256=sha(before), stage_sha256=sha(after),
    requested_common_marker_launcher_exists=(ROOT/'src/arena_bringup/launch/marker_course.launch.py').exists(),
    original_source_unchanged=SOURCE.read_bytes()==before,
    scope='Windows stage only; source/log preserved; no ROS/WSL/Gazebo/Jetson/build execution')
(A / 'source_evidence.json').write_text(json.dumps(evidence, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps({k:v for k,v in evidence.items() if k not in ('original_error','actual_command')}, ensure_ascii=False))
