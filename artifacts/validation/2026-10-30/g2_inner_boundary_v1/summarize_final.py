"""저장된 시험 결과만 다시 읽어 표를 만든다. 새 주행이나 제어 계산은 없다."""
import json
from pathlib import Path

BASE=Path(__file__).resolve().parent


def load(path):
    return json.loads((BASE/path).read_text(encoding='utf-8'))


def table(batch):
    rows=[]
    for c in load(f'{batch}/report.json')['cases']:
        f=c['first_bypass']
        rows.append(dict(name=c['case']['name'],x=c['final_x'],y=c['final_y'],contact=c['contact'],gap=round(c['gap_min'],4),
            target_gap=round(c['target_gap_min'],4),grass=c['grass_max'],passed=c['passed_t'],rejoin=c['rejoin_t'],
            centered=c['centered_t'],first=None if not f else [f['t'],round(f['offset'],4),f['transition']],
            phase=c['last_phase'],stop_zero=c['stop_zero'],final_speed=c['final_speed'],ms_mean=round(c['ms_mean'],1),ms_p95=c['ms_p95'],ms_max=c['ms_max'],
            safety=c['safety']))
    return rows


def sustained_zero(rows,start,length=12):
    """cut-in 시작 뒤 연속 0명령이 이어진 첫 시각. 평소 명령 깜빡임과 구분한다."""
    for i,r in enumerate(rows):
        if r['t']<start:
            continue
        run=rows[i:i+length]
        if len(run)==length and all(x['cmd']==0. for x in run):
            return r
    return None


def timeline():
    out=[]
    for c in load('timeline1/report.json')['cases']:
        name=c['case']['name']
        if name=='control_no_motion':
            continue
        rows=load(f'timeline1/{name}/series.json')
        start=c['case']['scene']['start']
        zero=sustained_zero(rows,start)
        contact=c['contact'];stop=c['ego_stopped']
        if contact and stop and contact['t']-stop['t']>=.1:
            kind='ego_stopped_then_target_arrived'
        elif contact:
            kind='ego_still_moving_or_stopping_at_contact'
        else:
            kind='no_contact'
        out.append(dict(name=name,onset_t=start,onset_gap_m=c['onset']['target_gap_m'],onset_x=c['onset']['x'],
            detect_t=None if not c['detect'] else c['detect']['t'],detect_gap_m=None if not c['detect'] else c['detect']['target_gap_m'],
            sustained_zero_t=None if not zero else zero['t'],sustained_zero_gap_m=None if not zero else zero['gap'],
            ego_stop_t=None if not stop else stop['t'],ego_stop_gap_m=None if not stop else stop['target_gap_m'],ego_stop_x=None if not stop else stop['x'],
            contact_t=None if not contact else contact['t'],speed_at_contact=None if not contact else contact['speed'],
            stopped_before_contact_s=None if not (stop and contact) else round(contact['t']-stop['t'],2),kind=kind,
            passed_target=c['passed'],minimum_target_gap_m=c['minimum_target_gap_m'],
            shadow_ticks=c['shadow_compared'],shadow_existing_hold_ticks=c['shadow_existing_would_hold'],
            shadow_first_hold_t=None if not c['shadow_first_hold'] else c['shadow_first_hold']['t'],
            shadow_first_hold_x=None if not c['shadow_first_hold'] else c['shadow_first_hold']['x']))
    return out


def trace():
    rows=[]
    for c in load('trace2/report.json')['cases']:
        rows.append(dict(name=c['case']['name'],progress=c['final_forward_progress_m'],lateral=c['lateral_displacement_m'],
            contact=c['contact_samples'],gap=c['minimum_body_gap_m'],target_gap=c['target_gap_min_m'],grass=c['maximum_wheels_touching_grass'],
            passed=c['passed_target_t'],rejoin=c['rejoin_t'],centered=c['centered_t'],stop_zero=c['stop_command_all_zero'],
            final_speed=c['final_speed_mps'],guard_reduced_ticks=c['safety_reduced_ticks'],processing=c['processing_ms'],replay=c['replay'],trace_sha256=c['trace_sha256']))
    return rows


result=dict(final_seeds=[r for r in table('batch9') if r['name'].startswith('final_')],
    bumps=[r for r in table('batch9') if not r['name'].startswith('final_')],
    counterexamples=table('batch8'),comparison=table('batch7'),timeline=timeline(),trace=trace())
(BASE/'final_summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
for key in ('final_seeds','bumps'):
    print('==',key)
    for r in result[key]:
        print(r['name'],r['x'],r['y'],'ct',r['contact'],'gap',r['gap'],'tgap',r['target_gap'],'grass',r['grass'],'pass',r['passed'],'rej',r['rejoin'],'ctr',r['centered'],'first',r['first'],'phase',r['phase'],'stop',r['stop_zero'],'ms',r['ms_mean'],r['ms_p95'],r['ms_max'],dict((k[:6],v) for k,v in r['safety'].items()))
print('== timeline')
for r in result['timeline']:
    print(r['name'],'onset',r['onset_t'],r['onset_gap_m'],'det',r['detect_t'],r['detect_gap_m'],'zero',r['sustained_zero_t'],r['sustained_zero_gap_m'],'stop',r['ego_stop_t'],r['ego_stop_gap_m'],'x',r['ego_stop_x'],'ct',r['contact_t'],'v',r['speed_at_contact'],'lead',r['stopped_before_contact_s'],r['kind'],'pass',r['passed_target'],'min',r['minimum_target_gap_m'],'shadow',r['shadow_existing_hold_ticks'],'/',r['shadow_ticks'],r['shadow_first_hold_t'],r['shadow_first_hold_x'])
print('== trace')
for r in result['trace']:
    print(r['name'],r['progress'],r['lateral'],'ct',r['contact'],'gap',round(r['gap'],4),'tgap',round(r['target_gap'],4),'grass',r['grass'],'pass',r['passed'],'rej',r['rejoin'],'ctr',r['centered'],'stop',r['stop_zero'],r['final_speed'],'guard',r['guard_reduced_ticks'],{k:round(v,1) for k,v in r['processing'].items()},r['replay']['passed'])
