"""cwd의 scripts/profile_follow_trace.py를 __main__으로 실행한 뒤 프로젝트 모듈의 실제 파일 위치를 출력한다."""
import json
from pathlib import Path
import runpy
import sys

trace, out, repo = sys.argv[1:4]
sys.argv = ['scripts/profile_follow_trace.py', trace, '--input-mode', 'stream', '--warmups', '0', '--repeats', '1',
            '--output', out]
try:
    runpy.run_path('scripts/profile_follow_trace.py', run_name='__main__')
    code = 0
except SystemExit as stop:
    code = stop.code
root, repo = Path.cwd().resolve(), Path(repo).resolve()
here = Path(__file__).resolve()
project = {name: Path(module.__file__).resolve() for name, module in list(sys.modules.items())
           if getattr(module, '__file__', None) and Path(module.__file__).resolve().is_relative_to(repo)
           and Path(module.__file__).resolve() != here}
print(json.dumps(dict(cli_exit=code, project_modules=len(project),
                      from_extract_only=code == 0 and bool(project) and all(p.is_relative_to(root) for p in project.values()),
                      outside=sorted(name for name, p in project.items() if not p.is_relative_to(root)))))
