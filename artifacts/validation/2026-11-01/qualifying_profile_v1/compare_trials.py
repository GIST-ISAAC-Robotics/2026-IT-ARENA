"""같은 2×5 트랙·차량을 쓴 첫 랩만 표로 비교한다. 반복 신뢰도나 실차 승인이 아니다."""
import argparse
import json
from hashlib import sha256
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent;DAY=HERE.parent
sys.path.insert(0,str(DAY/'race_acceleration_v1'))
from compare import analyze

CASES={
    'base2':('official_prelap_v1','run2'),
    'turn2':('turn_preview_v1','run4'),
    'rise10':('race_acceleration_v1','run1'),
    'both10':('qualifying_profile_v1','run1'),
}
DIGEST=lambda p:sha256(p.read_bytes()).hexdigest()


def geometry(run):
    world=ET.parse(run/'runtime_world.sdf').getroot(); meshes={}
    # 배치 파일의 시험별 임시 디렉터리만 정규화한다. 실제 OBJ 내용도 별도 대조한다.
    for uri in world.iter('uri'):
        raw=uri.text or ''
        match=re.fullmatch(r'/tmp/arena_world_[^/]+/(batched_[^/]+\.obj)',raw)
        if match:
            name=match[1];meshes[name]=DIGEST(run/name);uri.text='archived_mesh/'+name
    if len(meshes)!=3:raise ValueError('노면·잔디·벽의 3개 배치 메시가 확인되지 않음')
    return dict(world_xml=sha256(ET.tostring(world)).hexdigest(),batch_meshes=meshes,
                vehicle_sdf=DIGEST(run/'runtime_vehicle.sdf'))


def score(keys):
    result={};reference=None
    for name in keys:
        folder,run_name=CASES[name];base=DAY/folder;run=base/run_name
        r=json.loads((run/'report.json').read_bytes());geo=geometry(run)
        if reference is None:reference=geo
        if geo!=reference:raise ValueError('실제 월드 또는 차량 기하가 달라 동일 입력 표에서 제외: '+name)
        surface=json.loads((base/(run_name+'_surface.json')).read_bytes())
        if not (r['team_signal_evidence']['passed'] and r['grid_slot']==3 and not r['speed_bump_enabled']
                and r['requested_lidar_rate_hz']==10 and r['lidar_acquisition']=='sequential'):
            raise ValueError('2×5 신호·3번 그리드·10 Hz·턱 조건이 다름')
        result[name]=dict(profile=r['speed_profile'],runtime=analyze(run),explicit_stable_stop=r['stop_stable'],
            raw_report_pass=r['passed'],source_unchanged=r['source_unchanged'],clean_shutdown=r['shutdown_clean'],
            collision_samples=r['collision_samples'],wheel_wall_samples=surface['wall_wheel_samples'],
            maximum_grass_wheels=surface['interpretations']['any']['max_off_wheels'],report_sha256=DIGEST(run/'report.json'))
    return dict(trials=result,shared_geometry=reference,
        scope='같은 물리 시설·차량의 단독 첫 랩. 서로 다른 실행의 초기 위상·렌더 부하·궤적까지 동일하지는 않다.')


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('out',type=Path);ap.add_argument('profiles',nargs='+',choices=list(CASES))
    a=ap.parse_args();value=score(a.profiles)
    with a.out.open('x',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2)
    print({k:round(v['runtime']['observed_s'],3) for k,v in value['trials'].items()})
