"""2행×5열 RGB 출발 단독 시험의 시설 입력·평가. 차량 노드에서 사용하지 않는다."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ART = ROOT/'artifacts/validation'
FIXTURE = ART/'2026-11-01/signal_marker_offset_v1/fixture'
DRIVER = ART/'2026-10-27/official_departure_v2'
MANIFEST = ART/'2026-10-29/signal_spec_1029_v1/fixture/manifest.json'
ENTRY = ART/'2026-11-01/official_prelap_v1/course.launch.py'
STAGES = ('green_probe','off','red1','red2','red3','red4','green')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fixture_inputs():
    geometry = FIXTURE.parent/'geometry.json'
    if digest(FIXTURE/'world.sdf') != json.loads(geometry.read_text())['output_sha256']:
        raise ValueError('ID45 배치가 검토한 월드와 다름')
    spec = json.loads(MANIFEST.read_text())
    old_path = DRIVER/'fixture/manifest.json'
    old = json.loads(old_path.read_text())
    channels = lambda d: [(v['name'],v['column']) for v in d['lamps']]
    if (len(spec['lamps']) != 10 or channels(old) != channels(spec)
            or spec['world_name'] != old['world_name']):
        raise ValueError('신호 발생기의 광원 채널과 시험 월드가 다름')
    world = ET.parse(FIXTURE/'world.sdf').getroot().find('world')
    for name,_ in channels(spec):
        if world.find(f"model/link[@name='{name}']/visual[@name='{name}_visual']") is None:
            raise ValueError('물리 월드 광원 누락: '+name)
    inputs = [geometry,MANIFEST,old_path,ENTRY,Path(__file__),DRIVER/'signal_driver.py',
              DRIVER/'signal_driver_v2.py',DRIVER/'native/build/material_client',
              ROOT/'src/arena_bringup/launch/simulation.launch.py',
              ROOT/'scripts/validate_basic_autonomy.py',
              *sorted(p for p in FIXTURE.rglob('*') if p.is_file())]
    return {p.relative_to(ROOT).as_posix():digest(p) for p in inputs}


def sequence_evidence(folder, labels, trace):
    """녹색 단독 시험점등은 출발 신호가 아니다. 실제 10등의 적용 회신과 차의 첫 허가를 비교."""
    path = Path(folder)/'signal_sequence.jsonl'
    rows = [r for line in path.read_text().splitlines() if (r := json.loads(line)).get('stage')]
    lamps = {v['name']:v['column'] for v in json.loads(MANIFEST.read_text())['lamps']}
    order = [r['stage'] for r in rows]
    receipts_ok = order == list(STAGES)
    for r in rows:
        stage = r['stage']; receipts = r.get('receipts',[])
        red = int(stage[-1]) if stage.startswith('red') else 0
        receipts_ok &= (len(receipts)==10 and {a.get('lamp') for a in receipts}==set(lamps)
                        and all(all(a.get(k) is True for k in ('parsed','executed','service_result','applied'))
                        and a.get('lit') is (lamps.get(a['lamp'],-1)<red or
                            (lamps.get(a['lamp'])==4 and stage in ('green','green_probe'))) for a in receipts))
    times = [r['applied_sim_s'] for r in rows]
    starts = [r['time'] for r in trace if r['autonomy'].get('started')]
    after = bool(starts and order==list(STAGES) and min(starts)>=times[-1])
    states = {'green_probe','red','green'}.issubset(labels)
    return dict(passed=bool(receipts_ok and states and after and
                             all(a<b for a,b in zip(times,times[1:]))),
                stages=order,ten_receipts_each=bool(receipts_ok),observed_probe_red_green=states,
                first_start_sim_s=min(starts,default=None),green_applied_sim_s=times[-1] if rows else None,
                start_after_actual_green=after)
