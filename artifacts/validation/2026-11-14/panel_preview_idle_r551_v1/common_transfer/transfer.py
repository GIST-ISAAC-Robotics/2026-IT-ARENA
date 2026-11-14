"""검토된 r551만 현재 파일에 이관하고 기존 변경을 보존한다."""
from pathlib import Path
import difflib
import hashlib
import json

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
STAGE = HERE.parent / 'stage'

def sha(data):
    return hashlib.sha256(data).hexdigest()

manifest = json.loads((STAGE / 'manifest.json').read_text(encoding='utf-8'))
changes = {}
for name, evidence in manifest.items():
    path = ROOT / 'src/arena_autonomy/arena_autonomy' / name
    before = path.read_bytes()
    assert sha(before) == evidence['before_sha256'], name
    stage = (STAGE / name).read_bytes()
    assert sha(stage) == evidence['stage_sha256'], name
    # 기존 공통 LF를 유지한다. 내용은 검토된 stage와 같다.
    changes[path] = (before, stage.replace(b'\r\n', b'\n'))

path = ROOT / 'tests/operator_panel_page_checks.js'
before = path.read_bytes()
text = before.decode('utf-8')
stage = (STAGE / 'page_checks.js').read_text(encoding='utf-8')
begin = stage.index('checks.hidden_tab_keeps_ops_poll_without_preview=')
end = stage.index('(async()=>{const name=process.argv[3]', begin)
extra = stage[begin:end]
anchor = '(async()=>{const name=process.argv[3]'
assert text.count(anchor) == 1 and 'checks.hidden_tab_keeps_ops_poll_without_preview=' not in text
newline = '\r\n' if b'\r\n' in before else '\n'
changes[path] = (before, text.replace(anchor, extra.replace('\n', newline) + anchor).encode('utf-8'))

path = ROOT / 'tests/test_operator_panel_page.py'
before = path.read_bytes()
text = before.decode('utf-8')
anchor = "    'preview_reconnect_clears_displayed_inputs',"
assert text.count(anchor) == 1
newline = '\r\n' if b'\r\n' in before else '\n'
extra = newline.join([
    anchor,
    "    'hidden_tab_keeps_ops_poll_without_preview',",
    "    'unselected_camera_does_not_fetch_and_discard',",
    "    'jpeg_finishing_while_hidden_is_not_displayed',",
])
changes[path] = (before, text.replace(anchor, extra).encode('utf-8'))

path = ROOT / 'tests/test_operator_preview_idle.py'
assert not path.exists(), path
text = (STAGE / 'test_routes.py').read_text(encoding='utf-8')
text = text.replace("HERE=Path(__file__).resolve().parent", "ROOT=Path(__file__).resolve().parents[1]")
text = text.replace("def handler(clock,ops,filename='race_ops_node.py'):\n    source=HERE/filename", "def handler(clock,ops):\n    source=ROOT/'src/arena_autonomy/arena_autonomy/race_ops_node.py'")
old = '''    before=handler(clock,state,'race_ops_node.py.before');stage=handler(clock,state)
    try:
        clock[0]=12.
        assert get(stage)==get(before)
        assert state.preview.lease_until==15.
        assert get(stage)[1]['preview']['lidar']['points']==[[1.,0.,1.]]'''
new = '''    h=handler(clock,state)
    try:
        clock[0]=12.
        code,row,headers=get(h)
        assert code==200 and headers==dict(X_Arena_Role='operator')
        assert state.preview.lease_until==15. and state.core.clients['owner']==12.
        expected=state.core.view(12.,12.)
        expected.update(manual=state.manual.snapshot(),preview=state.preview.snapshot())
        assert row==expected  # query 생략은 전체 상태와 미리보기 payload를 보존한다.
        assert row['preview']['lidar']['points']==[[1.,0.,1.]]
        assert row['preview']['camera']['sequence']==2
        h.path='/api/state?client=owner&preview=1'
        assert h.do_GET()==(code,row,headers)'''
assert text.count(old) == 1
text = text.replace(old, new)
changes[path] = (b'', text.encode('utf-8'))

record = {}
patch = []
for path, (before, after) in changes.items():
    relative = path.relative_to(ROOT).as_posix()
    if path.suffix == '.py':
        compile(after, str(path), 'exec')
    (HERE / (path.name + '.before')).write_bytes(before)
    (HERE / (path.name + '.after')).write_bytes(after)
    record[relative] = dict(before_sha256=sha(before) if before else None, after_sha256=sha(after))
    patch.extend(difflib.unified_diff(before.decode('utf-8').splitlines(True),
        after.decode('utf-8').splitlines(True), fromfile='a/'+relative, tofile='b/'+relative))

for path, (before, after) in changes.items():
    assert (path.read_bytes() if path.exists() else b'') == before, path
    path.write_bytes(after)
(HERE / 'manifest.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
(HERE / 'r551_common.patch').write_text(''.join(patch), encoding='utf-8')
print(json.dumps(record, indent=2))
