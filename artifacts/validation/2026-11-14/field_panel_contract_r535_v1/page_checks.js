// 실제 패널 script를 실행하는 로컬 DOM/HTTP 모형. 브라우저 렌더나 실차 증명이 아니다.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const source=fs.readFileSync(process.argv[2],'utf8');
const script=source.match(/<script>([\s\S]*)<\/script>/)[1];
function fixture(mode='autonomous'){
 const state={state:'signal',state_label:'신호 대기',vehicle_id:'car',binding:{session:'race',race_epoch:0,intervention:1},
  connection:{status_age_s:0,status_stale:false,mcu_age_s:0},race:{started:false},speeds:{},driving:{},mcu:{state:'DISARMED'},
  sensors:[],commands:[],events:[],log:{},controls:{},development:{enabled:true,limit_mps:1.5,maximum_mps:1.5},
  manual:{mode,state:'prepared',prepared:mode==='manual',limit_mps:.1,maximum_mps:1.5,pulse_limit_mps:.1,
   steering_limit_deg:11.459,duration_max_s:2,commands:[],binding:{session:'manual',generation:0}}};
 for(const op of ['manual_visual_start','recovery_reset','resume_start','prepare_start','new_race','restore_race','set_speed_limit'])state.controls[op]={allowed:true};
 return state;
}
function harness(){
 const elements={},requests=[];
 for(const match of source.matchAll(/<[^>]+\bid="([^"]+)"[^>]*>/g))elements[match[1]]={value:match[0].match(/\bvalue="([^"]*)"/)?.[1]||'',textContent:'',innerHTML:'',disabled:match[0].includes(' disabled'),checked:false,className:'',hidden:false,removeAttribute:()=>{}};
 elements.lidarcanvas.getContext=()=>new Proxy({}, {get:()=>()=>{}});
 const context={document:{getElementById:id=>elements[id]},crypto:{getRandomValues:a=>a.fill(3)},
  sessionStorage:{getItem:()=>'',setItem:()=>{}},performance:{now:()=>10000},setInterval:()=>{},URL:{revokeObjectURL:()=>{}},
  fetch:(url,options)=>new Promise((resolve,reject)=>requests.push({url,options,resolve,reject})),console};
 vm.createContext(context);vm.runInContext(script,context);
 const run=code=>vm.runInContext(code,context);
 context.fixture=fixture();run("auth={vehicle:'car',token:'old',readonly:false};lastOk=10000;render(fixture)");
 return {elements,requests,context,run};
}
const response=(state,status=200,role='operator')=>({ok:status===200,status,headers:{get:()=>role},json:async()=>state});
const flush=()=>new Promise(resolve=>setImmediate(resolve));
const checks={
 async units_and_pulse(){
  const h=harness(),e=h.elements;
  e.speedlimitkmh.value='3.6';e.speedlimitkmh.oninput();assert.equal(Number(e.speedlimit.value),1);
  h.run('render(fixture)');assert.equal(Number(e.speedlimit.value),1);assert.equal(e.speedlimitcurrent.textContent,'1.500 m/s / 5.400 km/h');
  e.speedlimit.value='.1';e.speedlimit.oninput();assert.equal(Number(e.speedlimitkmh.value),.36);
  assert(e.manualprepare.disabled&&e.manualrun.disabled);
  h.context.fixture=fixture('manual');h.run('render(fixture)');assert.equal(e.manualrun.disabled,false);assert(e.manualstart.disabled);
  e.manualspeedkmh.value='.18';e.manualspeedkmh.oninput();e.manualseconds.value='.5';e.manualseconds.oninput();
  assert.equal(Number(e.manualspeed.value),.05);assert.equal(e.manualdistance.textContent,'0.025 m');
  e.manualangle.value='5';e.manualrun.onclick();
  const sent=JSON.parse(h.requests[0].options.body);assert.equal(h.requests[0].url,'/api/manual');assert.deepEqual(sent.value,{speed_mps:.05,seconds:.5,steering_deg:5});
 },
 async auth_failure_disables_controls(){
  for(const status of [401,409,500]){
   const h=harness();assert.equal(h.elements.manualstart.disabled,false);
   const task=h.run('poll()');h.requests[0].resolve(response({error:'fixture'},status));await task;
   for(const id of ['manualstart','start','prepare','setspeedlimit','manualprepare','manualrun'])assert(h.elements[id].disabled,`${status}: ${id}`);
   assert.equal(h.elements.stop.disabled,false);const before=h.requests.length;h.run("send('manual_visual_start','SIGNAL_CONFIRMED')");
   assert.equal(h.requests.length,before,'오래된 수신 성공 시간을 사용해 출발 요청하지 않는다');
  }
 },
 async reconnect_resets_binding_and_cap(){
  const h=harness(),e=h.elements;e.speedlimit.value='.25';e.speedlimit.oninput();
  e.vehicle.value='new-car';e.token.value='new-token';e.login.onclick();
  assert.equal(h.run('last'),null);assert(e.manualstart.disabled&&e.setspeedlimit.disabled);assert.equal(e.speedlimitcurrent.textContent,'미확인');
  const next=fixture();next.vehicle_id='new-car';next.development.limit_mps=.1;
  h.requests[0].resolve(response(next));await flush();assert.equal(Number(e.speedlimit.value),.1);assert.equal(Number(e.speedlimitkmh.value),.36);
 },
 async late_old_connection_is_ignored(){
  const h=harness();const oldTask=h.run('poll()');
  h.elements.vehicle.value='new-car';h.elements.token.value='new-token';h.elements.login.onclick();
  const next=fixture();next.vehicle_id='new-car';next.development.limit_mps=.1;
  h.requests[1].resolve(response(next));await flush();
  h.requests[0].resolve(response(fixture(),200,'view'));await oldTask;
  assert.equal(h.elements.vid.textContent,'new-car');assert.equal(h.run('auth.readonly'),false);assert.equal(h.elements.speedlimitcurrent.textContent,'0.100 m/s / 0.360 km/h');
 },
 async late_old_snapshot_does_not_restore_wait_green(){
  const h=harness(),older=h.run('poll()'),newer=h.run('poll()');
  const running=fixture();running.state='running';running.state_label='주행';running.race.started=true;running.controls.manual_visual_start.allowed=false;
  h.requests[1].resolve(response(running));await newer;assert(h.elements.manualstart.disabled);
  h.requests[0].resolve(response(fixture()));await older;assert(h.elements.manualstart.disabled);assert.equal(h.elements.badge.textContent,'주행');
 },
 async old_failure_does_not_break_new_connection(){
  const h=harness(),oldTask=h.run('poll()');h.elements.vehicle.value='new-car';h.elements.token.value='new-token';h.elements.login.onclick();
  const next=fixture();next.vehicle_id='new-car';h.requests[1].resolve(response(next));await flush();
  h.requests[0].reject(Error('old connection closed'));await oldTask;assert.equal(h.elements.conn.className,'');assert.equal(h.elements.manualstart.disabled,false);
 },
 async stop_remains_available_without_snapshot(){
  const h=harness();h.elements.vehicle.value='car';h.elements.token.value='new-token';h.elements.login.onclick();
  h.elements.stop.onclick();const request=h.requests[1];assert.equal(request.url,'/api/command');
  assert.equal(JSON.parse(request.options.body).op,'stop');assert.equal(JSON.parse(request.options.body).view,null);
 },
 async old_command_receipt_does_not_overwrite_new_connection(){
  const h=harness(),oldTask=h.run("sendManual('prepare')");h.elements.vehicle.value='new-car';h.elements.token.value='new-token';h.elements.login.onclick();
  h.elements.manualresult.textContent='새 연결';h.requests[0].resolve(response({stage:'rejected',reason:'old receipt'}));await oldTask;
  assert.equal(h.elements.manualresult.textContent,'새 연결');assert.equal(h.requests.length,2);
 }
};

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
