"""/api/command STOP의 기존 수동 전달 확인. 소켓/ROS/실제 자식 실행 없음."""
import ast
import io
import json
from pathlib import Path
from types import SimpleNamespace as N
import tempfile
import hashlib
from urllib.parse import urlsplit

from arena_autonomy.race_ops_core import token
from test_operator_panel_manual import fake_node, prepare, submit


root = Path(__file__).resolve().parents[4]
source = root/'src/arena_autonomy/arena_autonomy/race_ops_node.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
handler_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Handler')
namespace = dict(BaseHTTPRequestHandler=object, json=json, token=token, urlsplit=urlsplit,
                 MAX_BODY=4096, time=N(monotonic=lambda: 100.))
exec(compile(ast.Module(body=[handler_class], type_ignores=[]), str(source), 'exec'), namespace)

rows = []
for phase in ('preparing', 'prepared', 'queued_run', 'running'):
    with tempfile.TemporaryDirectory(prefix='arena_r505_stop_') as temporary:
        node = fake_node.__wrapped__(Path(temporary))
        node.published = []
        node.publish = lambda payload: node.published.append(payload)
        if phase == 'preparing':
            submit(node.manual, 'prepare')
            node.tick_manual(100.)
            assert node.manual_prepare is not None
        else:
            prepare(node)
            if phase in ('queued_run', 'running'):
                submit(node.manual, 'run', 'pulse', dict(speed_mps=.05, seconds=.5, steering_deg=0.))
                if phase == 'running':
                    node.tick_manual(100.)
                    assert node.manual.process is not None
        process = node.manual.process

        # Handler.do_POST itself selects /api/command -> OpsCore.submit with no view.
        handler = object.__new__(namespace['Handler'])
        body = json.dumps(dict(op='stop', request='reconnect-stop', client='desk', view=None)).encode()
        handler.path = '/api/command'
        handler.headers = {'Content-Length': str(len(body))}
        handler.rfile = io.BytesIO(body)
        handler.server = N(ops=node)
        handler.role = lambda: 'operator'  # already-authenticated operator; no socket or token lookup
        receipts = []
        handler.respond = lambda code, data, **kwargs: receipts.append((code, json.loads(data)))
        handler.do_POST()
        assert receipts[0][0] == 200 and receipts[0][1]['stage'] == 'queued'
        entry = node.core.next_action()
        assert entry['op'] == 'stop' and entry['view'] is None
        node.execute(entry)
        node.poll_calls(100.)

        assert node.manual_prepare is None and node.manual.prepared_identity is None
        assert not node.manual.queue
        assert node.service_calls[-1] == 'stop'
        assert node.core.mcu['state'] == 'ESTOP_LATCHED'
        assert node.published == [dict(v=1, request='reconnect-stop', op='stop')]
        if phase == 'queued_run':
            assert node.manual.entries['pulse']['stage'] == 'cancelled'
        if phase == 'running':
            assert process.stdin.closed and process.stdin.recorded.endswith('STOP\n')
            assert node.manual.state == 'stopping'
            process.complete()
            node.tick_manual(100.)
            assert node.manual.process is None and node.manual.state == 'idle'
        rows.append(dict(phase=phase, api='/api/command', view=None, http_status=200,
                         prepared_cleared=True, queued_run_cancelled=phase == 'queued_run',
                         child_stop_stdin_closed=phase == 'running', mcu_stop_calls=node.service_calls.count('stop'),
                         race_stop_published=True, fake_child_collected=node.manual.process is None))

result = dict(passed=True, cases=rows,
              boundary='Actual Handler and RaceOps methods, mocked ROS services and child; no physical output or sockets',
              runtime_source_changed=False, physical_stop_verified=False,
              source_sha256={p: hashlib.sha256((root/p).read_bytes()).hexdigest() for p in [
                  'src/arena_autonomy/arena_autonomy/race_ops_node.py',
                  'src/arena_autonomy/arena_autonomy/race_ops_manual.py',
                  'src/arena_autonomy/arena_autonomy/race_ops_core.py']})
Path(__file__).with_name('reconnect_stop_result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps(result, ensure_ascii=False, indent=2))
