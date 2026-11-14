// DOM 동작과 입력 단위 변환의 정적 UI 검사. 브라우저 렌더/ROS 전송 검증과 구분한다.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(process.argv[2],'utf8');
const script=html.match(/<script>([\s\S]*)<\/script>/)[1];
const elements={};
for(const match of html.matchAll(/id="([^"]+)"/g))elements[match[1]]={value:'',textContent:'',innerHTML:'',disabled:false,checked:false,className:'',hidden:false,removeAttribute(name){delete this[name]}};
let draws=0,points=0;const canvas={clearRect:()=>draws++,beginPath(){},arc(){},stroke(){},fillText(){},moveTo(){},lineTo(){},fillRect:()=>points++};
elements.lidarcanvas.width=640;elements.lidarcanvas.height=480;elements.lidarcanvas.getContext=()=>canvas;
elements.lidarcanvas.getBoundingClientRect=()=>({left:0,top:0,width:640,height:480});
elements.previewrange.value='3';elements.previewkind.value='native';elements.previewlidartopic.value='/scan/native';elements.previewcameratopic.value='/camera/color/image_raw';
const context={document:{getElementById:id=>elements[id]},crypto:{getRandomValues:a=>a.fill(3)},
 sessionStorage:{getItem:()=>'',setItem:()=>{}},performance:{now:()=>100},setInterval:()=>{},fetch:()=>Promise.reject('offline'),console};
vm.createContext(context);vm.runInContext(script,context);
const run=code=>vm.runInContext(code,context);
run("auth={readonly:false};lastOk=100");
const state={state:'setup',state_label:'준비',vehicle_id:'car',binding:{},connection:{status_age_s:0,status_stale:false,mcu_age_s:0},
 race:{},speeds:{plan_mps:.1,protected_mps:.09,mcu_target_mps:.08,motor_shaft_equiv_mps:.07},driving:{},mcu:{state:'DISARMED'},
 sensors:[],commands:[],events:[],log:{},controls:{},development:{enabled:true,limit_mps:1.5,maximum_mps:1.5},
 manual:{mode:'autonomous',state:'idle',pulse_limit_mps:.2,steering_limit_deg:11.459,commands:[]}};
for(const op of ['manual_visual_start','recovery_reset','resume_start','prepare_start','new_race','restore_race','set_speed_limit'])state.controls[op]={allowed:true};
context.fixture=state;run('render(fixture)');
assert(elements.speedlimitbox.hidden===false);
assert(elements.speedlimitcurrent.textContent==='1.500 m/s / 5.400 km/h');
assert(elements.speeds.innerHTML.includes('0.100 m/s / 0.360 km/h'));
assert(elements.manualprepare.disabled&&elements.manualrun.disabled);
elements.speedlimitkmh.value='3.6';elements.speedlimitkmh.oninput();
assert(Number(elements.speedlimit.value)===1);
run('render(fixture)');assert(Number(elements.speedlimit.value)===1); // 상태 갱신이 미적용 편집값을 덮지 않는다.
assert(elements.speedlimitcurrent.textContent.startsWith('1.500'));
elements.speedlimit.value='.25';elements.speedlimit.oninput();assert(Number(elements.speedlimitkmh.value)===.9);
elements.manualspeedkmh.value='.54';elements.manualspeedkmh.oninput();assert(Number(elements.manualspeed.value)===.15);
elements.manualseconds.value='.5';elements.manualseconds.oninput();assert(elements.manualdistance.textContent==='0.075 m');
elements.manualspeed.value='.1';elements.manualspeed.oninput();assert(Number(elements.manualspeedkmh.value)===.36);
run("sendManual=(op,value)=>{globalThis.sent={op,value}};");
state.manual={mode:'manual',state:'prepared',prepared:true,limit_mps:.15,maximum_mps:1.5,pulse_limit_mps:.15,steering_limit_deg:11.459,commands:[],binding:{session:'test',generation:0}};
run('render(fixture)');assert(!elements.manualrun.disabled&&elements.start.disabled);
elements.manualspeed.value='.2';elements.manualseconds.value='.5';elements.manualangle.value='5';
elements.manualrun.onclick();assert(context.sent===undefined); // 초과값을 줄여 실행하지 않는다.
elements.manualspeed.value='.1';elements.manualrun.onclick();
assert(context.sent.op==='run'&&context.sent.value.speed_mps===.1&&context.sent.value.steering_deg===5);
assert(html.includes('예상 이동거리(등속 가정)'));
(async()=>{
 context.URL={createObjectURL:()=> 'blob:fixture',revokeObjectURL(){}};
 state.preview={config:{lidar:false,camera:false},active:false,errors:{},lidar:null,camera:null};
 run('render(fixture)');assert(elements.lidarmeta.textContent==='OFF');const firstDraws=draws;
 run('render(fixture)');assert(draws===firstDraws); // OFF일 때 반복 canvas 작업 없음.
 state.preview.config.lidar=true;state.preview.active=true;
 state.preview.lidar={sequence:1,frame:'fixture_raw',receipt_age_s:.1,drawn_count:2,raw_count:5,invalid_count:3,points:[[1,0,1],[4,0,4]],stale:false};
 run('render(fixture)');assert(points===1);assert(elements.lidarmeta.textContent.includes('무효/무반환 3'));
 state.preview.lidar={...state.preview.lidar,stale:true};run('render(fixture)');assert(elements.lidarmeta.textContent.startsWith('오래됨'));
 elements.previewrange.value='6';elements.previewrange.oninput();assert(points===4); // 새 범위에 두 점 표시.
 elements.previewkind.value='laser_scan';elements.previewkind.onchange();assert(elements.previewlidartopic.value==='/scan');
 const posts=[];context.fetch=async(path,options)=>{posts.push({path,options});return {ok:true,json:async()=>state.preview,blob:async()=>({type:'image/jpeg'})}};
 run('poll=()=>{};auth={readonly:true,token:"view",vehicle:"fixture"}');
 elements.previewlidar.checked=true;elements.previewcamera.checked=false;await run('configurePreview()');
 assert(posts[0].path==='/api/preview'&&posts[0].options.headers['X-Arena-Token']==='view');
 assert(JSON.parse(posts[0].options.body).config.lidar_type==='laser_scan');
 state.preview.config.camera=true;elements.previewcamera.checked=true;await run('cameraPreview({sequence:2})');
 assert(elements.camerapreview.src==='blob:fixture'&&!elements.camerapreview.hidden);
 run('setStale("fixture offline")');assert(elements.camerapreview.hidden&&elements.previewstate.textContent.includes('연결 미확인'));
 await run('configurePreview(true)');assert(!elements.previewlidar.checked&&!elements.previewcamera.checked&&elements.camerapreview.hidden);
 assert(html.includes('기본 OFF')&&html.includes('무반환/NaN은 빈 공간이라는 뜻이 아닙니다'));
 console.log(JSON.stringify({passed:true,r429_units_and_manual_regression:true,preview_canvas_zoom:true,off_no_repeat_draw:true,stale_visible:true,authenticated_config:true,jpeg_toggle:true}));
})().catch(error=>{console.error(error);process.exitCode=1});
