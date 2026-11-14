// DOM 동작과 입력 단위 변환의 정적 UI 검사. 브라우저 렌더/ROS 전송 검증과 구분한다.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(process.argv[2],'utf8');
const script=html.match(/<script>([\s\S]*)<\/script>/)[1];
const elements={};
for(const match of html.matchAll(/id="([^"]+)"/g))elements[match[1]]={value:'',textContent:'',innerHTML:'',disabled:false,checked:false,className:'',hidden:false};
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
console.log(JSON.stringify({passed:true,bidirectional_units:true,edited_value_preserved:true,server_value_separate:true,autonomous_manual_disabled:true,no_silent_clamp:true}));
