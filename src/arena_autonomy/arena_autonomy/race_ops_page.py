"""운영 화면 한 장. 속도·조향 입력 없이 상태와 네 가지 명령만 다룬다."""
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
<section><h2>조작 <span class="muted">(속도·조향 입력 없음, Wi-Fi STOP은 HW 정지 대체 아님)</span></h2>
<button id="stop">STOP</button>
<div class="row"><button id="reset">복구 RESET</button><button id="start">재개 START</button></div>
<p class="muted" style="margin:4px 0">잠깐 정지했다면 START로 재개합니다. 차량 재배치·상태 초기화가 필요하면 먼저 RESET을 사용합니다. RESET만으로 출발하지 않습니다.</p>
<div id="why"></div>
<div class="row" style="margin-top:4px"><button id="prepare">최초 출발 준비(해제·ARM)</button></div>
<div id="restorebox" hidden><p class="muted" id="restoretext"></p><label><input type="checkbox" id="restoreok" style="width:auto"> 이 기록이 현재 경기임을 확인합니다</label> <button id="restore">같은 경기 복구</button></div>
<details><summary>새 경기(출발 사실 삭제)</summary><label><input type="checkbox" id="newok" style="width:auto"> 기존 출발 사실을 지우고 새 신호를 기다립니다</label> <button id="newrace">새 경기 시작</button></details>
</section>
<section><h2>속도 m/s <span class="muted">(읽기 전용)</span></h2><table id="speeds"></table>
<p class="muted" style="margin:3px 0 0">모터축 환산은 단일 모터축 카운트 환산값이며 차체 실제 속도가 아닙니다.</p></section>
<section><h2>명령 결과</h2><table id="commands"></table><div class="muted">실제 차체 정지: <b id="phys">미확인</b></div></section>
<section><h2>주행 상태</h2><table id="driving"></table></section>
<section><h2>센서 갱신 나이 s</h2><table id="sensors"></table></section>
<section><h2>최근 사건</h2><div id="list"></div><div class="row" style="margin-top:4px"><a id="csv" href="#">CSV</a> <a id="html" href="#">HTML</a> <span class="muted" id="logstate"></span></div></section>
</main>
<script>
"use strict";
const $=id=>document.getElementById(id);
const client=Array.from(crypto.getRandomValues(new Uint8Array(8)),b=>b.toString(16).padStart(2,'0')).join('');
let auth=null,last=null,lastOk=0,busy=false;
const rid=()=>Array.from(crypto.getRandomValues(new Uint8Array(12)),b=>b.toString(16).padStart(2,'0')).join('');
const fmt=(v,d=2)=>v===null||v===undefined?'-':typeof v==='number'?v.toFixed(d):String(v);
const esc=s=>String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
const headers=()=>({'X-Arena-Token':auth.token,'X-Arena-Vehicle':auth.vehicle,'Content-Type':'application/json'});
const STAGE={queued:'접수',sent:'전송, 응답 대기',policy_accepted:'정책 수락',policy_rejected:'정책 거절',rejected_by_adapter:'어댑터 거절',unknown_after_send:'결과 미확인',preparing:'준비 중',permitted:'주행 허가',cancelled:'취소'};
const OPS={stop:'STOP',recovery_reset:'복구 RESET',resume_start:'재개 START',prepare_start:'출발 준비',new_race:'새 경기',restore_race:'같은 경기 복구'};
function rows(id,list){$(id).innerHTML=list.map(r=>'<tr>'+r.map(c=>c).join('')+'</tr>').join('')}
function setStale(text){$('conn').textContent=text;$('conn').className='stale'}
function render(s){
 last=s;$('badge').textContent=s.state_label;$('badge').className=s.state;$('vid').textContent=s.vehicle_id;
 const b=s.binding,age=s.connection.status_age_s;
 $('race').textContent='경기 '+String(b.race_id||'-').slice(0,8)+' · 번호 '+fmt(b.race_epoch,0)+' · 개입 '+fmt(b.intervention,0)+' · 세션 '+String(b.session||'-').slice(0,8)+(s.race.start_source?' · 출발 '+s.race.start_source:'');
 if(s.connection.status_stale)setStale('차량 상태 오래됨 '+fmt(age,1)+'s');else{$('conn').textContent='상태 '+fmt(age,2)+'s 전';$('conn').className=''}
 const sp=s.speeds;rows('speeds',[['<td>계획(/drive)</td>','<td class=n>'+fmt(sp.plan_mps)+'</td>'],['<td>보호 후(/drive/safe)</td>','<td class=n>'+fmt(sp.protected_mps)+'</td>'],['<td>MCU 적용 목표</td>','<td class=n>'+fmt(sp.mcu_target_mps)+'</td>'],['<td>모터축 환산</td>','<td class=n>'+fmt(sp.motor_shaft_equiv_mps)+(sp.motor_feedback_valid===false?' <span class=bad>무효</span>':'')+'</td>']]);
 const d=s.driving,m=s.mcu;rows('driving',[['<td>상태</td><td>'+esc(d.state)+' / '+esc(d.behavior??'-')+'</td>'],['<td>이유</td><td>'+esc(d.reason??'-')+'</td>'],['<td>기준 벽</td><td>'+esc(d.side??'-')+(d.side_marker!=null?' (마커 '+d.side_marker+')':' (마커 근거 없음)')+' · 마커 '+esc(JSON.stringify(d.marker_ids??[]))+'</td>'],['<td>보호층</td><td>'+esc(d.safety_reason??'-')+' · 스윕 여유 '+fmt(d.clearance_m)+' m</td>'],['<td>정책</td><td>'+esc(s.race.policy_reason??'-')+' / '+esc(s.race.actuator_reason??'-')+'</td>'],['<td>MCU</td><td>'+esc(m.state??'-')+' ('+esc(m.reason??'-')+') 세대 '+fmt(m.generation,0)+(m.estop_held?' <span class=bad>HW 정지 입력</span>':'')+'</td>'],['<td>START 단계</td><td>'+(s.race.resume?esc(s.race.resume.phase)+' 대기 '+esc(s.race.resume.missing.join(',')||'-'):s.race.last_resume?esc(s.race.last_resume.outcome)+': '+esc(s.race.last_resume.reason):'-')+'</td>']]);
 rows('sensors',s.sensors.map(x=>['<td>'+x.name+'</td>','<td class="n '+(x.ok?'ok':'bad')+'">'+fmt(x.age_s,3)+'</td>','<td class="n muted">&lt; '+x.limit_s+'</td>']));
 rows('commands',s.commands.slice(0,5).map(c=>['<td>'+(OPS[c.op]||c.op)+'</td>','<td>'+esc(STAGE[c.stage]||c.stage)+(c.adapter_reason?' '+esc(c.adapter_reason):'')+(c.policy?' · '+esc(c.policy.reason):'')+(c.mcu?' · MCU '+esc(c.mcu):'')+'</td>']));
 $('list').innerHTML=s.events.map(e=>'<div>'+fmt(e.sim_s,2)+' <span class=muted>'+esc(e.kind)+'</span> '+esc(e.text)+'</div>').join('');
 $('logstate').textContent=s.log&&s.log.dropped?'기록 누락 '+s.log.dropped:'';
 const c=s.controls,unknown=s.commands.some(x=>x.stage==='queued'||x.stage==='sent'),off=k=>!c[k].allowed||unknown||busy||auth.readonly;
 $('reset').disabled=off('recovery_reset');$('start').disabled=off('resume_start');$('prepare').disabled=off('prepare_start');
 $('newrace').disabled=off('new_race')||!$('newok').checked;$('stop').disabled=auth.readonly;
 $('why').textContent=auth.readonly?'읽기 전용 접속입니다.':s.connection.status_stale?'상태가 오래되어 STOP 외 명령을 막았습니다.':s.race.resume?'START 처리 중: 아직 주행 허가가 아닙니다.':(!c.resume_start.allowed&&s.race.latch?'START 불가: '+c.resume_start.reason:'');
 const o=s.restore_offer;$('restorebox').hidden=!o;
 if(o){$('restoretext').textContent='저장된 경기 '+o.race_id.slice(0,8)+' · 기준 벽 '+o.side+' · 저장 '+new Date(o.saved_wall_time*1000).toLocaleTimeString()+'. 출발 신호를 새로 본 것이 아닙니다.';$('restore').disabled=off('restore_race')||!$('restoreok').checked}
}
async function poll(){
 if(!auth)return;
 try{const r=await fetch('/api/state?client='+client,{headers:headers(),cache:'no-store'});
  if(r.status===401||r.status===409){setStale('인증/차량 불일치 '+r.status);return}
  const s=await r.json();auth.readonly=r.headers.get('X-Arena-Role')==='view';lastOk=performance.now();render(s)
 }catch(e){setStale('연결 끊김: 상태·명령 결과 미확인 '+fmt((performance.now()-lastOk)/1000,1)+'s');
  ['reset','start','prepare','newrace','restore'].forEach(k=>$(k).disabled=true)}
}
async function send(op,confirm){
 const stale=!last||performance.now()-lastOk>1500;
 if(op!=='stop'&&stale)return;
 busy=op!=='stop';const body={op,request:rid(),client,view:last?last.binding:null,confirm:confirm||null};
 try{const r=await fetch('/api/command',{method:'POST',headers:headers(),body:JSON.stringify(body)});if(!r.ok)$('why').textContent='요청 거절 '+r.status}
 catch(e){$('why').textContent='응답 없음: 결과 미확인. 상태를 다시 확인하십시오. 자동 재전송하지 않습니다.'}
 busy=false;poll()
}
$('stop').onclick=()=>send('stop');$('reset').onclick=()=>send('recovery_reset');$('start').onclick=()=>send('resume_start');$('prepare').onclick=()=>send('prepare_start');
$('newrace').onclick=()=>{send('new_race','NEW_RACE');$('newok').checked=false};$('restore').onclick=()=>{send('restore_race','SAME_RACE');$('restoreok').checked=false};
function exportLink(id,kind){$(id).onclick=async e=>{e.preventDefault();const r=await fetch('/api/export.'+kind+'?seconds=120',{headers:headers()});const u=URL.createObjectURL(await r.blob());const a=document.createElement('a');a.href=u;a.download='arena_events.'+kind;a.click();URL.revokeObjectURL(u)}}
exportLink('csv','csv');exportLink('html','html');
$('login').onclick=()=>{auth={vehicle:$('vehicle').value.trim(),token:$('token').value,readonly:false};$('authmsg').textContent='연결 시도';sessionStorage.setItem('arena_vehicle',auth.vehicle);poll().then(()=>{$('authmsg').textContent=''})};
$('vehicle').value=sessionStorage.getItem('arena_vehicle')||'';
setInterval(poll,200);
</script></body></html>
"""
