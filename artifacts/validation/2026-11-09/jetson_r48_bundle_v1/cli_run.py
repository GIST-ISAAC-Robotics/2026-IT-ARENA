"""대상 루트의 문서상 profile_follow_trace.py CLI를 __main__으로 1회 실행하고 명령 열·실제 import 위치를 기록한다.

python3 -I -B 로 실행한다. 인자: <대상 루트> <trace> <CLI 출력 폴더> <기록 JSON(.json.gz)> <저장소 루트>.
기록 래퍼는 최외곽 command() 반환값을 그대로 돌려주며 판단에 관여하지 않는다(비용 수치에는 부하가 포함된다).
"""
import gzip
import hashlib
import json
from pathlib import Path
import runpy
import sys

root, trace, out, record, repo = (Path(a).resolve() for a in sys.argv[1:6])
start_path = list(sys.path)
sys.path[:0] = [str(root/'src/arena_autonomy'), str(root/'src/arena_vehicle_interface'), str(root/'scripts')]

from arena_autonomy.follow_trace import comparable_decision, freeze  # noqa: E402
from arena_autonomy.local_follow import LocalFollow  # noqa: E402
from arena_autonomy.bypass_handoff import BypassHandoffFollow  # noqa: E402

calls, depth = [], [0]


def recorded(method):
    def wrapper(self, *args, **kwargs):
        depth[0] += 1
        try:
            output = method(self, *args, **kwargs)
        except Exception as error:
            # 호출자가 follow_input_stop으로 바꾸는 예외도 순서를 지키도록 같은 열에 남긴다.
            if depth[0] == 1:
                text = f'raise:{type(error).__name__}:{error}'
                calls.append([0., 0., 'raise:'+type(error).__name__, hashlib.sha256(text.encode()).hexdigest()])
            raise
        finally:
            depth[0] -= 1
        if depth[0] == 0:
            frozen = comparable_decision(freeze(output))
            text = json.dumps(frozen, sort_keys=True, ensure_ascii=False, allow_nan=False)
            calls.append([frozen[0], frozen[1], frozen[2].get('reason'), hashlib.sha256(text.encode()).hexdigest()])
        return output
    return wrapper


LocalFollow.command = recorded(LocalFollow.command)
BypassHandoffFollow.command = recorded(BypassHandoffFollow.command)

script = root/'scripts/profile_follow_trace.py'
sys.argv = [str(script), str(trace), '--input-mode', 'stream', '--warmups', '0', '--repeats', '1', '--output', str(out)]
try:
    runpy.run_path(str(script), run_name='__main__')
    code = 0
except SystemExit as stop:
    code = stop.code if isinstance(stop.code, int) else 1

here = Path(__file__).resolve()
project, outside = {}, []
for name, module in list(sys.modules.items()):
    path = getattr(module, '__file__', None)
    if not path:
        continue
    path = Path(path).resolve()
    if path == here:
        continue
    if path.is_relative_to(repo):
        project[name] = path
        if not path.is_relative_to(root):
            outside.append(name)
    elif name.split('.')[0] in ('arena_autonomy', 'arena_vehicle_interface'):
        outside.append(name)
sequence = hashlib.sha256('\n'.join(c[3] for c in calls).encode()).hexdigest()
with gzip.open(record, 'xt', encoding='utf-8') as stream:
    json.dump(dict(calls=calls), stream)
summary = dict(cli_exit=code, isolated=sys.flags.isolated, no_user_site=sys.flags.no_user_site,
               cwd_on_start_path='' in start_path or str(Path.cwd()) in start_path,
               start_path=start_path,
               project_modules=len(project), from_root_only=bool(project) and not outside, outside=sorted(outside),
               recorded_calls=len(calls), command_sequence_sha256=sequence)
print(json.dumps(summary, ensure_ascii=False))
raise SystemExit(code)
