"""예약 적용 시험: ACK/원시/출처/기본 경로와 반복 물리를 따로 감사한다."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from arena_vehicle_interface.evaluation_schedule import audit_scheduled, TOPIC, EVENT_TOPIC
from analyze_evaluator_gc import run_result, sha
from analyze_initial_startup import world_and_model, normalized_xml, readiness_evidence
from evaluation_startup import audit_startup, topic_connections
from analyze_apply_trace import analyze_run as trace_run, divergence
from analyze_scheduled_phase import compare_metrics

ROOT=Path(__file__).resolve().parents[1]


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def scheduled_readiness(record):
    try:
        ready=record['readiness']
        return all(ready[t]['rc']==0 and topic_connections(ready[t]['stdout'])['publishers']==1
                   and topic_connections(ready[t]['stdout'])['subscribers']>=1 for t in (TOPIC,EVENT_TOPIC))
    except (KeyError,TypeError):return False


def model_without_schedule(path):
    model=ET.parse(path).getroot()
    for plugin in model.iter('plugin'):
        for tag in ('evaluation_scheduled_apply','evaluation_wall_timeout'):
            for item in plugin.findall(tag):plugin.remove(item)
    return normalized_xml(model)


def applied_window(events,start,end):
    # UUID는 실행별 값이다. 내부 epoch 정합은 ACK 감사에서 별도로 검사한다.
    fields=('seq','stamp_us','apply_us','sim_us','speed','steering')
    return [{k:e[k] for k in fields} for e in events
            if e.get('type')=='applied' and start<=e['sim_us']<=end]


def compare_envelopes(a,b,start,end):
    x,y=applied_window(a,start,end),applied_window(b,start,end)
    first=next((i for i,(p,q) in enumerate(zip(x,y)) if p!=q),None)
    equal=bool(x) and x==y
    return {'same_all_normal_applied_envelopes':equal,'counts':[len(x),len(y)],
            'first_difference_index':first if first is not None else (min(len(x),len(y)) if len(x)!=len(y) else None),
            'first_different_rows':None if first is None else [x[first],y[first]],
            'scope':'동일 명령 시작~프로필 종료+0.2초. 같은 stamp/속도/조향/예정 및 실제 step. STOP 비동기 수신은 별도.'}


def terminal_order_valid(events):
    """플러그인의 STOP ACK 뒤에는 새 ARM 전 정상 적용이 없어야 한다."""
    active=False
    for e in events:
        if e.get('type')=='armed':active=True
        elif e.get('type') in ('stopped','fault'):active=False
        elif e.get('type')=='applied' and not active:return False
    return bool(events) and not active


def analyze_run(run,contract,binary):
    report=json.loads((run/'report.json').read_text())
    evidence=run_result(run,contract,binary)
    trace,data,_,_=trace_run(run,contract,binary)
    shape=world_and_model(run,report)
    startup=audit_startup(report['evaluation_startup'],rows(run/'plugin_command_events.jsonl'))
    enabled=report.get('evaluation_scheduled_apply_requested') is True
    reader=report.get('startup_reader_audit',{})
    checks={'evidence':evidence['valid'],'gc_policy':evidence['checks']['gc_policy'],
            'shape':shape['passed'],'startup':startup['passed'],
            'first_command_zero':startup.get('first_update',{}).get('speed')==0
                and startup.get('first_update',{}).get('steering')==0,
            'reader_clean':reader.get('handled_messages',0)>=2 and reader.get('parse_errors')==0
                and reader.get('handler_errors')==0,
            'readiness':readiness_evidence(report['evaluation_startup'])}
    events=[];audit=None
    if enabled:
        raw={name:rows(run/f'{name}.jsonl') for name in ('scheduled_sent','scheduled_events','scheduled_states')}
        checks['scheduled_readiness']=scheduled_readiness(report['evaluation_startup'])
        checks['scheduled_raw_hashes']=all(sha(run/f'{name}.jsonl')==report[name]['sha256']
            and len(value)==report[name]['samples'] for name,value in raw.items())
        audit=audit_scheduled(raw['scheduled_sent'],raw['scheduled_events'],raw['scheduled_states'])
        checks['scheduled_ack']=audit['passed']
        checks['audit_recomputed_matches']=audit==report['scheduled_apply_audit']
        events=raw['scheduled_events']
        checks['stop_terminal_order']=terminal_order_valid(events)
    checks['posthoc_trace_run']=trace['validity']['passed']
    return {'name':run.name,'scheduled':enabled,'passed':all(checks.values()),'checks':checks,
            'evidence':evidence,'shape':shape,'startup':startup,'ack':audit,
            'start_ns':report['command_schedule']['start_ns'],'duration_s':report['command_profile']['duration_s'],
            'trace':trace,'positive_apply_events':sum(e.get('type')=='applied' and e['speed']>0 for e in events),
            'stop_events':[e for e in events if e.get('type')=='stopped']},data,events


def analyze(base,contract,binary):
    names=('n1_native','t1_scheduled','t2_scheduled')
    result=[analyze_run(base/name,contract,binary) for name in names]
    native,a,b=[r[0] for r in result]
    common=('source_sha256','runtime_vehicle_sha256','world_normalized_sha256','meshes_sha256','profile','spawn_pose')
    checks={'distinct_runs':a['name']!=b['name'],'both_valid':a['passed'] and b['passed'],
            'both_scheduled':a['scheduled'] and b['scheduled'],'start_equal':a['start_ns']==b['start_ns'],
            'initial_step_equal':a['trace']['trace']['spawn']==b['trace']['trace']['spawn'],
            **{'same_'+k:a['evidence'][k]==b['evidence'][k] for k in common}}
    start=a['start_ns']//1000;end=start+round(a['duration_s']*1e6)+200000
    envelopes=compare_envelopes(result[1][2],result[2][2],start,end)
    metrics,within,secondary=compare_metrics(a['trace'],b['trace'])
    div=divergence(result[1][1],result[2][1],a['start_ns']) if checks['start_equal'] else None
    old=ROOT/'artifacts/validation/2026-10-11/initial_startup_v2/e3_embedded'
    native_checks={'native_valid':native['passed'],'native_disabled':not native['scheduled'],
        'native_model_matches_previous':normalized_xml(ET.parse(base/names[0]/'runtime_vehicle.sdf').getroot())
            ==normalized_xml(ET.parse(old/'runtime_vehicle.sdf').getroot()),
        'same_model_except_evaluation_tags':model_without_schedule(base/names[0]/'runtime_vehicle.sdf')
            ==model_without_schedule(base/names[1]/'runtime_vehicle.sdf'),
        'same_world_without_vehicle':native['shape']['world_without_vehicle_sha256']==a['shape']['world_without_vehicle_sha256'],
        'same_meshes':native['evidence']['meshes_sha256']==a['evidence']['meshes_sha256']}
    return {'runs':[r[0] for r in result],'native_checks':native_checks,'native_passed':all(native_checks.values()),
        'pair_checks':checks,'pair_comparable':all(checks.values()),'envelope_comparison':envelopes,
        'scheduled_application_repeat_confirmed':all(checks.values()) and envelopes['same_all_normal_applied_envelopes'],
        'divergence':div,'metrics':metrics,'metrics_within_tolerance':within,'secondary_metrics':secondary,
        'limits':['20 ms를 의도적으로 추가한 평가 전용 출력 경로. 실차 성능/기본 제동 모델 채택 근거가 아니다.',
                  '두 실행은 통계적 재현성/물리 수렴 보장이 아니며 주행 전체/다중 차량 검증도 아니다.',
                  'STOP 취소 우선과 오류 입력은 단위 시험으로 별도 확인하며 실차 안전으로 확대하지 않는다.']}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,required=True)
    parser.add_argument('--contract',type=Path,default=ROOT/'config/tests/scheduled_command_r7_sha256.json')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    registration=json.loads((args.base/'registered_manifest.json').read_text())
    result=analyze(args.base,json.loads(args.contract.read_text()),registration['plugin_sha256'])
    with args.output.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps({k:result[k] for k in ('native_checks','pair_checks','envelope_comparison',
        'scheduled_application_repeat_confirmed','divergence','metrics_within_tolerance')},ensure_ascii=False,indent=2))
