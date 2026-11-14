"""root 승인 범위의 page + 기존 DOM 검사만 현재 baseline에 통합한다."""
from pathlib import Path
import difflib
import hashlib
import json

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
PAGE=ROOT/'src/arena_autonomy/arena_autonomy/race_ops_page.py'
JS=ROOT/'tests/operator_panel_page_checks.js'
PY=ROOT/'tests/test_operator_panel_page.py'
expected='a4a30d4ee8100c6184e0f5bcace2209bbf54de0bae62dd27f773ad266cab0560'
before_page=PAGE.read_bytes();stage=(HERE/'race_ops_page.stage.py').read_bytes()
assert hashlib.sha256(before_page).hexdigest()==expected
before_text=before_page.decode('utf-8',errors='strict');stage_text=stage.decode('utf-8',errors='strict')
assert b'\xef\xbf\xbd' not in before_page+stage
assert before_text.split('<script>')[0].encode()==stage_text.split('<script>')[0].encode()
assert '\uc218\ub3d9 \ub2e8\ubc1c \uc870\uc791' in stage_text
assert '\uc2e0\ud638 \ud655\uc778 \ud6c4 \uc218\ub3d9 \ucd9c\ubc1c' in stage_text

runner='(async()=>{const name=process.argv[3]'
old_js=JS.read_text(encoding='utf-8')
stage_js=(HERE/'page_checks.js').read_text(encoding='utf-8')
base_js=old_js[:old_js.index(runner)]
assert stage_js.startswith(base_js)
extra=stage_js[len(base_js):stage_js.rindex(runner)]
assert 'preview_late_jpeg_does_not_cross_connection' not in old_js
new_js=base_js+extra+old_js[old_js.index(runner):]
old_py=PY.read_text(encoding='utf-8')
point="    'old_command_receipt_does_not_overwrite_new_connection',"
assert old_py.count(point)==1
new_py=old_py.replace(point,point+"\n    'preview_late_configuration_does_not_cross_connection',\n    'preview_late_jpeg_does_not_cross_connection',\n    'preview_reconnect_clears_displayed_inputs',")
target=HERE/'common_transfer';(target/'before').mkdir(parents=True,exist_ok=True);(target/'after').mkdir(exist_ok=True)
manifest=dict(utf8_strict_decode=True,replacement_character_present=False,static_html_bytes_identical=True,
    expected_korean_labels_present=True,files={},source_scope='page and existing DOM tests only',physical_commands_sent=0)
patch=[]
for path,data in [(PAGE,stage),(JS,new_js.encode()),(PY,new_py.encode())]:
    before=path.read_bytes()
    (target/'before'/path.name).write_bytes(before)
    path.write_bytes(data)
    (target/'after'/path.name).write_bytes(data)
    manifest['files'][str(path.relative_to(ROOT))]=dict(before_sha256=hashlib.sha256(before).hexdigest(),after_sha256=hashlib.sha256(data).hexdigest())
    patch.extend(difflib.unified_diff(before.decode().splitlines(True),data.decode().splitlines(True),
        fromfile='a/'+path.relative_to(ROOT).as_posix(),tofile='b/'+path.relative_to(ROOT).as_posix()))
(target/'r535_common.patch').write_text(''.join(patch),encoding='utf-8')
(target/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps(manifest,indent=2))
