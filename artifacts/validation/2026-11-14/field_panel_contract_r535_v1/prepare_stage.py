"""현재 공통 소스를 보존하고 미리보기 연결 식별 수정만 stage한다. 공통 파일은 쓰지 않는다."""
from pathlib import Path
import difflib
import hashlib
import json

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SOURCE = ROOT/'src/arena_autonomy/arena_autonomy/race_ops_page.py'
raw = SOURCE.read_bytes()
assert hashlib.sha256(raw).hexdigest() == 'a4a30d4ee8100c6184e0f5bcace2209bbf54de0bae62dd27f773ad266cab0560', 'stage baseline differs; preserve saved before/after'
before = raw.decode('utf-8')
(HERE/'race_ops_page.before.py').write_bytes(raw)
after = before
replacements = [
    (' previewImageBusy=true;', ' const connection=auth;\n previewImageBusy=true;'),
    ("{headers:headers(),cache:'no-store'});if(!response.ok)throw Error('JPEG '+response.status);",
     "{headers:headers(connection),cache:'no-store'});if(auth!==connection)return;if(!response.ok)throw Error('JPEG '+response.status);"),
    ("const blob=await response.blob();if(!last?.preview?.config.camera", "const blob=await response.blob();if(auth!==connection||!last?.preview?.config.camera"),
    ("}catch(error){$('camerameta').textContent='미리보기 오류: '+error.message}",
     "}catch(error){if(auth===connection)$('camerameta').textContent='미리보기 오류: '+error.message}"),
    (" if(off){$('previewlidar')", " const connection=auth;\n if(off){$('previewlidar')"),
    ("fetch('/api/preview',{method:'POST',headers:headers(),", "fetch('/api/preview',{method:'POST',headers:headers(connection),"),
    ("const result=await response.json();if(!response.ok)throw Error(result.error||response.status);renderPreview(result);poll()",
     "const result=await response.json();if(auth!==connection)return;if(!response.ok)throw Error(result.error||response.status);renderPreview(result);poll()"),
    ("catch(error){$('previewstate').textContent='미리보기 설정 오류: '+error.message}",
     "catch(error){if(auth===connection)$('previewstate').textContent='미리보기 설정 오류: '+error.message}"),
    (" $('badge').textContent='연결 확인 중';",
     " clearCamera();previewLidar=null;previewLidarSequence=null;drawLidar();\n $('previewstate').textContent='연결 확인 중';$('lidarmeta').textContent='새 연결 입력 대기';$('camerameta').textContent='새 연결 입력 대기';\n $('badge').textContent='연결 확인 중';"),
]
for old, new in replacements:
    assert after.count(old) == 1, old
    after = after.replace(old, new)
(HERE/'race_ops_page.stage.py').write_text(after, encoding='utf-8', newline='\n')
(HERE/'preview_connection.patch').write_text(''.join(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
    fromfile='a/src/arena_autonomy/arena_autonomy/race_ops_page.py', tofile='b/src/arena_autonomy/arena_autonomy/race_ops_page.py')), encoding='utf-8')

base = (ROOT/'tests/operator_panel_page_checks.js').read_text(encoding='utf-8')
base = base[:base.index('(async()=>{const name=process.argv[3]')]
extra = r'''
function previewRow(on=false){return {config:{lidar:false,camera:on,lidar_type:'native',lidar_topic:'/scan/native',camera_topic:'/rgb'},active:on,errors:{},lidar:null,camera:null}}
checks.preview_late_configuration_does_not_cross_connection=async()=>{
 const h=harness();h.elements.previewcamera.checked=true;
 const pending=h.run('configurePreview()');
 h.elements.vehicle.value='new-car';h.elements.token.value='new-token';h.elements.login.onclick();
 const next=fixture();next.vehicle_id='new-car';next.preview=previewRow(false);
 h.requests[1].resolve(response(next));await flush();assert.equal(h.elements.previewstate.textContent,'OFF');
 h.requests[0].resolve(response(previewRow(true)));await pending;
 assert.equal(h.elements.previewstate.textContent,'OFF','previous connection cannot restore preview ON');
 assert.equal(h.requests.length,2,'previous receipt must not create a new poll');
};
checks.preview_late_jpeg_does_not_cross_connection=async()=>{
 const h=harness();h.context.URL.createObjectURL=blob=>blob.old?'blob:old-car':'blob:new-car';
 h.elements.previewcamera.checked=true;h.context.fixture.preview=previewRow(true);h.run('render(fixture)');
 const pending=h.run('cameraPreview({sequence:12})');
 h.elements.vehicle.value='new-car';h.elements.token.value='new-token';h.elements.login.onclick();
 const next=fixture();next.vehicle_id='new-car';next.preview=previewRow(true);
 h.requests[1].resolve(response(next));await flush();
 h.run('cameraPreview({sequence:13})');assert.equal(h.requests.length,2,'old pending image keeps one request at a time');
 h.requests[0].resolve({ok:true,status:200,blob:async()=>({old:true})});await pending;
 assert.equal(h.elements.camerapreview.hidden,true,'previous JPEG cannot become new vehicle image');
 assert.equal(h.run('previewCameraSequence'),null);
 assert.equal(h.run('previewImageBusy'),false);
 const fresh=h.run('cameraPreview({sequence:13})');assert.equal(h.requests.length,3);
 assert.equal(h.requests[2].options.headers['X-Arena-Token'],'new-token');
 h.requests[2].resolve({ok:true,status:200,blob:async()=>({old:false})});await fresh;
 assert.equal(h.elements.camerapreview.src,'blob:new-car');assert.equal(h.elements.camerapreview.hidden,false);
};
checks.preview_reconnect_clears_displayed_inputs=async()=>{
 const h=harness();h.context.URL.createObjectURL=()=> 'blob:old-car';
 h.elements.previewcamera.checked=true;h.context.fixture.preview=previewRow(true);h.run('render(fixture)');
 const pending=h.run('cameraPreview({sequence:12})');
 h.requests[0].resolve({ok:true,status:200,blob:async()=>({old:true})});await pending;
 assert.equal(h.elements.camerapreview.hidden,false);
 h.run('previewLidar={sequence:7,points:[[1,0,1]],stale:false};previewLidarSequence=7');
 h.elements.vehicle.value='new-car';h.elements.token.value='new-token';h.elements.login.onclick();
 assert.equal(h.elements.camerapreview.hidden,true,'login immediately removes old image');
 assert.equal(h.run('previewLidar'),null);assert.equal(h.run('previewLidarSequence'),null);
};
(async()=>{const name=process.argv[3];assert(checks[name],name);await checks[name]();console.log(JSON.stringify({check:name,passed:true,scope:'Windows actual page script with local DOM/HTTP mock; no socket or ROS'}))})().catch(error=>{console.error(error);process.exitCode=1});
'''
(HERE/'page_checks.js').write_text(base+extra, encoding='utf-8')
manifest = dict(source=str(SOURCE.relative_to(ROOT)), before_sha256=hashlib.sha256(raw).hexdigest(),
    stage_sha256=hashlib.sha256((HERE/'race_ops_page.stage.py').read_bytes()).hexdigest(),
    production_changed=False, local_server_started=False, physical_commands_sent=0, wsl_jetson_access=False)
(HERE/'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
print(json.dumps(manifest, indent=2))
