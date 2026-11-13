"""운영 상한과 별도 수동 구성의 제한 조작 화면. 경기 자율 모드와 구분한다."""
PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>IT ARENA 운영</title>
<style>
:root{color-scheme:dark}*{box-sizing:border-box}
body{margin:0;background:#14171b;color:#e6e6e6;font:13px/1.35 system-ui,'Malgun Gothic',sans-serif}
header{display:flex;gap:10px;align-items:center;flex-wrap:wrap;padding:6px 10px;background:#1e2329}
#badge{font-size:19px;font-weight:700;padding:3px 12px;border-radius:4px;background:#555}
#badge.running{background:#1f7a3a}#badge.operator_stop,#badge.fault{background:#a12626}
#badge.resumable{background:#2460a8}#badge.recovery{background:#a8761a}#badge.yield_hold,#badge.signal,#badge.setup{background:#5b5f66}
#conn{padding:2px 8px;border-radius:4px;background:#26502f}#conn.stale{background:#a12626;font-weight:700}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(285px,1fr));gap:7px;padding:7px}
section{background:#1b1f24;border:1px solid #30363d;border-radius:4px;padding:5px 8px;min-width:0}
h2{margin:0 0 3px;font-size:12px;color:#9fb1c4;font-weight:600}
table{width:100%;border-collapse:collapse}td,th{padding:1px 4px;text-align:left;vertical-align:top;overflow-wrap:anywhere}td:first-child{white-space:nowrap}
td.n{text-align:right;font-variant-numeric:tabular-nums}.bad{color:#ff8a80;font-weight:700}.ok{color:#8fd19e}.muted{color:#8a939c}
button{font:inherit;font-weight:700;padding:9px 12px;border:0;border-radius:4px;color:#fff;background:#3a6ea5;cursor:pointer}
button:disabled{background:#333;color:#777;cursor:not-allowed}
#stop{background:#c62828;width:100%;font-size:21px;padding:14px;margin-bottom:6px}
.row{display:flex;gap:6px;flex-wrap:wrap}.row button{flex:1 1 40%}
details{margin-top:6px}summary{cursor:pointer;color:#9fb1c4}
input{font:inherit;background:#0e1114;color:#eee;border:1px solid #444;padding:3px 5px;width:150px}
#list{max-height:230px;overflow:auto;font-size:12px}#auth{padding:6px 10px;background:#26211a}
#why{min-height:17px;color:#ffcc80;font-size:12px}
</style></head><body>
<div id="auth">차량 <input id="vehicle" placeholder="차량 이름"> 토큰 <input id="token" type="password" placeholder="토큰"> <button id="login">연결</button> <span id="authmsg" class="muted"></span></div>
<header><span id="badge">연결 전</span><b id="vid">-</b><span id="conn">-</span><span id="race" class="muted"></span></header>
<main>
<section><h2>조작 <span class="muted">(Wi-Fi STOP은 HW 정지 대체 아님)</span></h2>
<button id="stop">STOP</button>
<div class="row"><button id="reset">복구 RESET</button><button id="start">재개 START</button></div>
<p class="muted" style="margin:4px 0">잠깐 정지했다면 START로 재개합니다. 차량 재배치·상태 초기화가 필요하면 먼저 RESET을 사용합니다. RESET만으로 출발하지 않습니다.</p>
<div id="why"></div>
<div id="speedlimitbox"><p>서버 적용 상한: <b id="speedlimitcurrent">미확인</b></p>
<div class="row"><label>편집 m/s <input id="speedlimit" type="number" min="0.05" step="0.01" aria-label="편집 속도 상한 m/s"></label><label>편집 km/h <input id="speedlimitkmh" type="number" min="0.18" step="0.036" aria-label="편집 속도 상한 km/h"></label></div>
<button id="setspeedlimit" disabled>상한 적용</button> <span id="speededitstate" class="muted">서버값 대기</span>
<p class="muted">STOP과 MCU 비ARM 상태에서 변경합니다. 적용만으로 출발하지 않으며 실제 차체 속도를 보장하지 않습니다. 초기 상한은 주행 목표가 아닙니다.</p></div>
<div class="row" style="margin-top:4px"><button id="prepare">최초 출발 준비(해제·ARM)</button></div>
<div class="row" style="margin-top:4px"><button id="manualstart" disabled>신호 확인 후 수동 출발</button></div>
<p class="muted" style="margin:4px 0">출발 신호를 직접 확인했을 때만 누릅니다. 카메라 검출 대신 운영자 수동 확인으로 기록됩니다. 버튼 수락은 즉시 구동을 보장하지 않으며 기존 센서·MCU·보호 조건은 유지됩니다.</p>
<div id="restorebox" hidden><p class="muted" id="restoretext"></p><label><input type="checkbox" id="restoreok" style="width:auto"> 이 기록이 현재 경기임을 확인합니다</label> <button id="restore">같은 경기 복구</button></div>
<details><summary>새 경기(출발 사실 삭제)</summary><label><input type="checkbox" id="newok" style="width:auto"> 기존 출발 사실을 지우고 새 신호를 기다립니다</label> <button id="newrace">새 경기 시작</button></details>
</section>
<section><h2>속도 m/s · km/h <span class="muted">(읽기 전용)</span></h2><table id="speeds"></table>
<p class="muted" style="margin:3px 0 0">모터축 환산은 단일 모터축 카운트 환산값이며 차체 실제 속도가 아닙니다.</p></section>
<section><h2>명령 결과</h2><table id="commands"></table><div class="muted">실제 차체 정지: <b id="phys">미확인</b></div></section>
<section><h2>주행 상태</h2><table id="driving"></table></section>
<section><h2>센서 갱신 나이 s</h2><table id="sensors"></table></section>
<section><h2>최근 사건</h2><div id="list"></div><div class="row" style="margin-top:4px"><a id="csv" href="#">CSV</a> <a id="html" href="#">HTML</a> <span class="muted" id="logstate"></span></div></section>
<section><h2>수동 단발 조작</h2><p id="manualmode" class="muted">별도 수동 실행 구성에서 사용할 수 있습니다.</p>
<p>단발 유효 상한: <b id="manualceiling">-</b> · 최대 <b id="manualduration">2</b>초</p>
<div class="row"><label>목표 m/s <input id="manualspeed" type="number" min="0.001" step="0.01" value="0.15"></label><label>목표 km/h <input id="manualspeedkmh" type="number" min="0.0036" step="0.036" value="0.54"></label></div>
<div class="row"><label>실행시간 s <input id="manualseconds" type="number" min="0.05" max="2" step="0.05" value="0.5"></label><label>중앙 등가 바퀴각 ° <input id="manualangle" type="number" step="0.1" value="0"></label></div>
<p class="muted">+는 좌회전, −는 우회전입니다. 서보축 각도가 아닙니다. 설정 범위 <b id="manualanglerange">-</b>이며 실제 장착 각도 검증값이 아닙니다.</p>
<p>예상 이동거리(등속 가정) v×t: <b id="manualdistance">0.075 m</b><br><span class="muted">가속·제동·미끄러짐·보호층 제한을 반영하지 않는 계산값입니다. 실제 이동거리·정지거리·곡선 순변위를 보장하지 않습니다.</span></p>
<div class="row"><button id="manualprepare" disabled>수동 준비(해제·ARM)</button><button id="manualrun" disabled>입력값으로 실행</button><button id="manualstop" disabled>수동 중지 / STOP</button></div>
<p id="manualresult" class="muted">자동 ARM·자동 재실행은 하지 않습니다. 종료 시 0목표·명시 STOP을 요청합니다.</p>
</section>
</main>
<script>
"use strict";
const $=id=>document.getElementById(id);
const client=Array.from(crypto.getRandomValues(new Uint8Array(8)),b=>b.toString(16).padStart(2,'0')).join('');
let auth=null,last=null,lastOk=0,busy=false,speedDirty=false,speedInitialized=false;
const rid=()=>Array.from(crypto.getRandomValues(new Uint8Array(12)),b=>b.toString(16).padStart(2,'0')).join('');
const fmt=(v,d=2)=>v===null||v===undefined?'-':typeof v==='number'?v.toFixed(d):String(v);
const speedText=v=>typeof v==='number'?fmt(v,3)+' m/s / '+fmt(v*3.6,3)+' km/h':'미확인';
function convertSpeed(from,to,factor){const raw=$(from).value,v=Number(raw);$(to).value=raw!==''&&Number.isFinite(v)?String(Number((v*factor).toFixed(9))):''}
function distancePreview(){const v=Number($('manualspeed').value),t=Number($('manualseconds').value);$('manualdistance').textContent=Number.isFinite(v*t)&&v>0&&t>0?fmt(v*t,3)+' m':'입력값 확인'}
const esc=s=>String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
const headers=()=>({'X-Arena-Token':auth.token,'X-Arena-Vehicle':auth.vehicle,'Content-Type':'application/json'});
const STAGE={queued:'접수',sent:'전송, 응답 대기',policy_accepted:'정책 수락',policy_rejected:'정책 거절',rejected_by_adapter:'어댑터 거절',unknown_after_send:'결과 미확인',preparing:'준비 중',permitted:'주행 허가',cancelled:'취소'};
const OPS={stop:'STOP',recovery_reset:'복구 RESET',resume_start:'재개 START',prepare_start:'출발 준비',manual_visual_start:'신호 확인 후 수동 출발',new_race:'새 경기',restore_race:'같은 경기 복구',set_speed_limit:'운영 속도 상한'};
function rows(id,list){$(id).innerHTML=list.map(r=>'<tr>'+r.map(c=>c).join('')+'</tr>').join('')}
function setStale(text){$('conn').textContent=text;$('conn').className='stale'}
function render(s){
 last=s;$('badge').textContent=s.state_label;$('badge').className=s.state;$('vid').textContent=s.vehicle_id;
 const b=s.binding,age=s.connection.status_age_s;
 $('race').textContent='경기 '+String(b.race_id||'-').slice(0,8)+' · 번호 '+fmt(b.race_epoch,0)+' · 개입 '+fmt(b.intervention,0)+' · 세션 '+String(b.session||'-').slice(0,8)+(s.race.start_source?' · 출발 '+s.race.start_source:'');
 if(s.connection.status_stale)setStale('차량 상태 오래됨 '+fmt(age,1)+'s');else{$('conn').textContent='상태 '+fmt(age,2)+'s 전';$('conn').className=''}
 const sp=s.speeds;rows('speeds',[['<td>계획(/drive)</td>','<td class=n>'+speedText(sp.plan_mps)+'</td>'],['<td>보호 후(/drive/safe)</td>','<td class=n>'+speedText(sp.protected_mps)+'</td>'],['<td>MCU 적용 목표</td>','<td class=n>'+speedText(sp.mcu_target_mps)+'</td>'],['<td>모터축 환산</td>','<td class=n>'+speedText(sp.motor_shaft_equiv_mps)+(sp.motor_feedback_valid===false?' <span class=bad>무효</span>':'')+'</td>']]);
 const d=s.driving,m=s.mcu;rows('driving',[['<td>상태</td><td>'+esc(d.state)+' / '+esc(d.behavior??'-')+'</td>'],['<td>이유</td><td>'+esc(d.reason??'-')+'</td>'],['<td>기준 벽</td><td>'+esc(d.side??'-')+(d.side_marker!=null?' (마커 '+d.side_marker+')':' (마커 근거 없음)')+' · 마커 '+esc(JSON.stringify(d.marker_ids??[]))+'</td>'],['<td>보호층</td><td>'+esc(d.safety_reason??'-')+' · 스윕 여유 '+fmt(d.clearance_m)+' m</td>'],['<td>정책</td><td>'+esc(s.race.policy_reason??'-')+' / '+esc(s.race.actuator_reason??'-')+'</td>'],['<td>MCU</td><td>'+esc(m.state??'-')+' ('+esc(m.reason??'-')+') 세대 '+fmt(m.generation,0)+(m.estop_held?' <span class=bad>HW 정지 입력</span>':'')+'</td>'],['<td>START 단계</td><td>'+(s.race.resume?esc(s.race.resume.phase)+' 대기 '+esc(s.race.resume.missing.join(',')||'-'):s.race.last_resume?esc(s.race.last_resume.outcome)+': '+esc(s.race.last_resume.reason):'-')+'</td>']]);
 rows('sensors',s.sensors.map(x=>['<td>'+x.name+'</td>','<td class="n '+(x.ok?'ok':'bad')+'">'+fmt(x.age_s,3)+'</td>','<td class="n muted">&lt; '+x.limit_s+'</td>']));
 rows('commands',s.commands.slice(0,5).map(c=>['<td>'+(OPS[c.op]||c.op)+'</td>','<td>'+esc(STAGE[c.stage]||c.stage)+(c.adapter_reason?' '+esc(c.adapter_reason):'')+(c.policy?' · '+esc(c.policy.reason):'')+(c.mcu?' · MCU '+esc(c.mcu):'')+'</td>']));
 $('list').innerHTML=s.events.map(e=>'<div>'+fmt(e.sim_s,2)+' <span class=muted>'+esc(e.kind)+'</span> '+esc(e.text)+'</div>').join('');
 $('logstate').textContent=s.log&&s.log.dropped?'기록 누락 '+s.log.dropped:'';
 const manual=s.manual||{},manualMode=manual.mode==='manual';
 const c=s.controls,unknown=s.commands.some(x=>x.stage==='queued'||x.stage==='sent'),off=k=>manualMode||!c[k].allowed||unknown||busy||auth.readonly;
 $('manualstart').disabled=off('manual_visual_start');
 $('reset').disabled=off('recovery_reset');$('start').disabled=off('resume_start');$('prepare').disabled=off('prepare_start');
 $('newrace').disabled=off('new_race')||!$('newok').checked;$('stop').disabled=auth.readonly;
 const dev=manualMode?manual:s.development;
 $('speedlimitcurrent').textContent=speedText(dev?.limit_mps);
 if(dev){$('speedlimit').max=dev.maximum_mps;$('speedlimitkmh').max=dev.maximum_mps*3.6;
  if(!speedInitialized){$('speedlimit').value=dev.limit_mps;convertSpeed('speedlimit','speedlimitkmh',3.6);speedInitialized=true}
  const edited=Number($('speedlimit').value),same=Number.isFinite(edited)&&Math.abs(edited-dev.limit_mps)<1e-9;
  $('speededitstate').textContent=same?'편집값 = 서버 적용값':'미적용 편집값';
  if(same)speedDirty=false;
 }
 const manualBusy=['preparing','running','stopping'].includes(manual.state)||(manual.commands||[]).some(x=>x.stage==='queued');
 const mcuFresh=typeof s.connection.mcu_age_s==='number'&&s.connection.mcu_age_s<.25;
 const stopped=['DISARMED','ESTOP_LATCHED','FAULT_LATCHED'].includes(m.state);
 $('setspeedlimit').disabled=!dev||busy||auth.readonly||(manualMode?manualBusy||!mcuFresh||!stopped:!c.set_speed_limit?.allowed||unknown);
 $('why').textContent=auth.readonly?'읽기 전용 접속입니다.':s.connection.status_stale?'상태가 오래되어 STOP 외 명령을 막았습니다.':s.race.resume?'START 처리 중: 아직 주행 허가가 아닙니다.':(!c.resume_start.allowed&&s.race.latch?'START 불가: '+c.resume_start.reason:'');
 const o=s.restore_offer;$('restorebox').hidden=!o;
 if(o){$('restoretext').textContent='저장된 경기 '+o.race_id.slice(0,8)+' · 기준 벽 '+o.side+' · 저장 '+new Date(o.saved_wall_time*1000).toLocaleTimeString()+'. 출발 신호를 새로 본 것이 아닙니다.';$('restore').disabled=off('restore_race')||!$('restoreok').checked}
 $('manualmode').textContent=manualMode?'별도 수동 구성 · '+manual.state+' · '+(manual.reason||''):'자율 구성: 수동 준비·실행 비활성. 자율 노드를 정상 종료한 뒤 guard·host·패널의 별도 수동 구성으로 실행하십시오.';
 $('manualceiling').textContent=speedText(manual.pulse_limit_mps);
 $('manualanglerange').textContent='±'+fmt(manual.steering_limit_deg,3)+'°';
 $('manualspeed').max=manual.pulse_limit_mps;$('manualspeedkmh').max=manual.pulse_limit_mps*3.6;
 $('manualangle').min=-manual.steering_limit_deg;$('manualangle').max=manual.steering_limit_deg;
 $('manualprepare').disabled=!manualMode||manualBusy||!mcuFresh||busy||auth.readonly;
 $('manualrun').disabled=!manualMode||!manual.prepared||manualBusy||busy||auth.readonly;
 $('manualstop').disabled=!manualMode||auth.readonly;
 const latestManual=manual.commands?.[0];if(latestManual)$('manualresult').textContent=latestManual.op+' · '+latestManual.stage+' · '+(latestManual.reason||'');
 if(manualMode){$('badge').textContent='수동 · '+manual.state;$('badge').className=manual.state==='running'?'running':'setup';
  if(mcuFresh){$('conn').textContent='MCU '+fmt(s.connection.mcu_age_s,3)+'s 전';$('conn').className=''}
  $('why').textContent='수동 구성에서는 경기 START/RESET·신호 수동 출발을 사용하지 않습니다.'}
 distancePreview();
}
async function poll(){
 if(!auth)return;
 try{const r=await fetch('/api/state?client='+client,{headers:headers(),cache:'no-store'});
  if(r.status===401||r.status===409){setStale('인증/차량 불일치 '+r.status);return}
  const s=await r.json();auth.readonly=r.headers.get('X-Arena-Role')==='view';lastOk=performance.now();render(s)
 }catch(e){setStale('연결 끊김: 상태·명령 결과 미확인 '+fmt((performance.now()-lastOk)/1000,1)+'s');
  ['reset','start','prepare','manualstart','newrace','restore','setspeedlimit','manualprepare','manualrun'].forEach(k=>$(k).disabled=true)}
}
async function send(op,confirm,value){
 const stale=!last||performance.now()-lastOk>1500;
 if(op!=='stop'&&stale)return;
 busy=op!=='stop';const body={op,request:rid(),client,view:last?last.binding:null,confirm:confirm||null};
 if(op==='set_speed_limit')body.value=value;
 try{const r=await fetch('/api/command',{method:'POST',headers:headers(),body:JSON.stringify(body)});if(!r.ok)$('why').textContent='요청 거절 '+r.status}
 catch(e){$('why').textContent='응답 없음: 결과 미확인. 상태를 다시 확인하십시오. 자동 재전송하지 않습니다.'}
 busy=false;poll()
}
async function sendManual(op,value){
 if(!auth||(op!=='stop'&&(!last||performance.now()-lastOk>1500)))return;
 busy=op!=='stop';
 try{const response=await fetch('/api/manual',{method:'POST',headers:headers(),body:JSON.stringify({op,request:rid(),client,view:last?.manual?.binding,value})});
  const receipt=await response.json();$('manualresult').textContent=receipt.stage+' · '+(receipt.reason||'요청 접수')}
 catch(e){$('manualresult').textContent='결과 미확인. 자동 재전송하지 않습니다.'}
 busy=false;poll();
}
$('stop').onclick=()=>last?.manual?.mode==='manual'?sendManual('stop'):send('stop');$('reset').onclick=()=>send('recovery_reset');$('start').onclick=()=>send('resume_start');$('prepare').onclick=()=>send('prepare_start');
$('manualstart').onclick=()=>{if(!busy&&!$('manualstart').disabled){$('manualstart').disabled=true;send('manual_visual_start','SIGNAL_CONFIRMED')}};
$('speedlimit').oninput=()=>{speedDirty=true;convertSpeed('speedlimit','speedlimitkmh',3.6);$('speededitstate').textContent='미적용 편집값'};
$('speedlimitkmh').oninput=()=>{speedDirty=true;convertSpeed('speedlimitkmh','speedlimit',1/3.6);$('speededitstate').textContent='미적용 편집값'};
$('setspeedlimit').onclick=()=>{const v=Number($('speedlimit').value),manual=last?.manual?.mode==='manual',dev=manual?last.manual:last?.development;
 if(dev&&Number.isFinite(v)&&v>=.05&&v<=dev.maximum_mps){if(manual)sendManual('set_speed_limit',v);else send('set_speed_limit',null,v)}else $('why').textContent='표시된 범위 안의 m/s 값을 입력하십시오.'};
$('manualspeed').oninput=()=>{convertSpeed('manualspeed','manualspeedkmh',3.6);distancePreview()};
$('manualspeedkmh').oninput=()=>{convertSpeed('manualspeedkmh','manualspeed',1/3.6);distancePreview()};
$('manualseconds').oninput=distancePreview;
$('manualprepare').onclick=()=>sendManual('prepare');$('manualstop').onclick=()=>sendManual('stop');
$('manualrun').onclick=()=>{const m=last?.manual,v=Number($('manualspeed').value),t=Number($('manualseconds').value),a=Number($('manualangle').value);
 if(!m||![v,t,a].every(Number.isFinite)||v<=0||v>m.pulse_limit_mps||t<.05||t>2||Math.abs(a)>m.steering_limit_deg){$('manualresult').textContent='입력값이 서버 범위를 벗어났습니다. 값을 줄여 실행하지 않습니다.';return}
 sendManual('run',{speed_mps:v,seconds:t,steering_deg:a})};
$('newrace').onclick=()=>{send('new_race','NEW_RACE');$('newok').checked=false};$('restore').onclick=()=>{send('restore_race','SAME_RACE');$('restoreok').checked=false};
function exportLink(id,kind){$(id).onclick=async e=>{e.preventDefault();const r=await fetch('/api/export.'+kind+'?seconds=120',{headers:headers()});const u=URL.createObjectURL(await r.blob());const a=document.createElement('a');a.href=u;a.download='arena_events.'+kind;a.click();URL.revokeObjectURL(u)}}
exportLink('csv','csv');exportLink('html','html');
$('login').onclick=()=>{auth={vehicle:$('vehicle').value.trim(),token:$('token').value,readonly:false};$('authmsg').textContent='연결 시도';sessionStorage.setItem('arena_vehicle',auth.vehicle);poll().then(()=>{$('authmsg').textContent=''})};
$('vehicle').value=sessionStorage.getItem('arena_vehicle')||'';
setInterval(poll,200);
</script></body></html>
"""
