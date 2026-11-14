"""미리보기 비표시 경로만 별도 복사본에 수정. 공통 소스는 읽기만 한다."""
from pathlib import Path
import difflib
import hashlib
import json

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
manifest={}
changes={
 'race_ops_page.py':[
  (' if(previewImageBusy||row.sequence===previewCameraSequence)return;',
   " if(document.hidden||!$('previewcamera').checked||previewImageBusy||row.sequence===previewCameraSequence)return;"),
  ("if(auth!==connection)return;if(!response.ok)throw Error('JPEG '+response.status);",
   "if(auth!==connection||document.hidden||!$('previewcamera').checked)return;if(!response.ok)throw Error('JPEG '+response.status);"),
  ('if(auth!==connection||!last?.preview?.config.camera',
   'if(auth!==connection||document.hidden||!last?.preview?.config.camera'),
  (' if(!p)return;const cfg=p.config;', ' if(!p||document.hidden)return;const cfg=p.config;'),
  ("fetch('/api/state?client='+client,{headers:headers(connection),cache:'no-store'})",
   "fetch('/api/state?client='+client+(document.hidden?'&preview=0':''),{headers:headers(connection),cache:'no-store'})"),
 ],
 'race_ops_node.py':[
  ("            if token(client):ops.preview.keepalive(client)",
   "            preview_requested = query.get('preview', ['1'])[0] != '0'\n            if token(client) and preview_requested:ops.preview.keepalive(client)"),
  ("            state['preview'] = ops.preview.snapshot()",
   "            state['preview'] = ops.preview.snapshot()\n            if not preview_requested:\n                state['preview'].update(lidar=None, camera=None)"),
 ]}
patch=[]
for name,replacements in changes.items():
 path=ROOT/'src/arena_autonomy/arena_autonomy'/name;raw=path.read_bytes();before=raw.decode();after=before
 for old,new in replacements:
  assert after.count(old)==1,old
  after=after.replace(old,new)
 compile(after,name,'exec')
 (HERE/(name+'.before')).write_bytes(raw)
 (HERE/name).write_text(after,encoding='utf-8')
 patch.extend(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='a/'+str(path.relative_to(ROOT)).replace('\\','/'),tofile='b/'+str(path.relative_to(ROOT)).replace('\\','/')))
 manifest[name]=dict(before_sha256=hashlib.sha256(raw).hexdigest(),stage_sha256=hashlib.sha256((HERE/name).read_bytes()).hexdigest())
(HERE/'preview_idle.patch').write_text(''.join(patch),encoding='utf-8')
(HERE/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
base=(ROOT/'tests/operator_panel_page_checks.js').read_text(encoding='utf-8')
base=base[:base.index('(async()=>{const name=process.argv[3]')]
extra=r'''
checks.hidden_tab_keeps_ops_poll_without_preview=async()=>{
 const h=harness();h.context.document.hidden=true;h.elements.previewcamera.checked=true;
 const task=h.run('poll()');
 assert(h.requests[0].url.includes('/api/state?client='));assert(h.requests[0].url.endsWith('&preview=0'));
 const next=fixture();next.preview=previewRow(true);next.preview.camera={sequence:22,frame:'camera',receipt_age_s:0,width:640,height:480,stale:false};
 h.requests[0].resolve(response(next));await task;
 assert.equal(h.requests.length,1,'no hidden JPEG request');assert.equal(h.elements.manualstart.disabled,false,'operation state still refreshed');
};
checks.unselected_camera_does_not_fetch_and_discard=async()=>{
 const h=harness();h.elements.previewcamera.checked=false;
 h.run('cameraPreview({sequence:1})');assert.equal(h.requests.length,0);
};
checks.jpeg_finishing_while_hidden_is_not_displayed=async()=>{
 const h=harness();let created=0;h.context.URL.createObjectURL=()=>{created++;return 'blob:camera'};
 h.elements.previewcamera.checked=true;h.context.fixture.preview=previewRow(true);h.run('render(fixture)');
 const task=h.run('cameraPreview({sequence:12})');assert.equal(h.requests.length,1);
 h.context.document.hidden=true;
 h.requests[0].resolve({ok:true,status:200,blob:async()=>({})});await task;
 assert.equal(created,0);assert.equal(h.run('previewCameraSequence'),null);
};
(async()=>{const name=process.argv[3];assert(checks[name],name);await checks[name]();console.log(JSON.stringify({check:name,passed:true,scope:'Windows DOM/HTTP mock; no network'}))})().catch(error=>{console.error(error);process.exitCode=1});
'''
(HERE/'page_checks.js').write_text(base+extra,encoding='utf-8')
print(json.dumps(manifest,indent=2))
