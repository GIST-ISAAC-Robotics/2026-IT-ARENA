"""사용 중인 경기 거리 시험 종료 후에만 검증한 격리 사본을 팀 소스에 옮긴다."""
import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
STAGE=ROOT/'build/solo_speed_envelope_stage'
ARCHIVE=HERE/'staged_deployment.zip'
BEFORE=HERE/'staged_deployment_before.zip'
PARENT='config/tests/current_lane_preference_r1_sha256.json'
CHILD='config/tests/solo_straight_budget_r1_sha256.json'
CHAIN='tests/test_source_provenance_r0_plus_b3.py'
NEW='src/arena_autonomy/arena_autonomy/solo_straight_budget.py'
TEST='tests/test_solo_straight_budget.py'
CAM='src/arena_description/models/arena_car/model.sdf.xacro'
sha=lambda b:hashlib.sha256(b).hexdigest()


def prepare():
    assert not ARCHIVE.exists()
    assert all(json.loads((HERE/f'staged_{m}.json').read_bytes())['mismatches']==0 for m in ('off','on'))
    checked=json.loads((HERE/'staged_source.json').read_bytes())
    with ZipFile(BEFORE) as previous:
        originals={n:previous.read(n) for n in previous.namelist()}
    targets={n:(STAGE/n).read_bytes() for n in [*originals,NEW,TEST]}
    # 카메라는 시뮬 센서 pose만 다르다. 다른 수정이 사후에 섞이면 중단.
    camera=targets[CAM].decode()
    tag='        <sensor name="chase_camera" type="camera">\n'
    a=camera.index(tag)+len(tag); b=camera.index('          <camera name="chase">',a)
    assert '0 -0.42 0</pose>' in camera[a:b] and (camera[:a]+camera[b:]).encode()==originals[CAM]
    for n,b in targets.items():
        if n!=CAM:
            assert sha(b)==checked[n]
    before={n:sha(b) for n,b in originals.items()}
    assert all(before[n]==sha((ROOT/n).read_bytes()) for n in before)
    assert not any((ROOT/n).exists() for n in (CHILD,NEW,TEST))
    contract=dict(schema_version=1,scope='단독 직선 속도 시험과 외부 녹화 구도. 기본 채택·실차 승인 아님.',
                  parent_contract=PARENT,parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
                  before_snapshot=BEFORE.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(BEFORE.read_bytes()),
                  approved_changes={},approved_evaluation_scripts={},new_dependencies={NEW:sha(targets[NEW])})
    for n in originals:
        group='approved_evaluation_scripts' if n.startswith('scripts/') else 'approved_changes'
        contract[group][n]=dict(before=before[n],after=sha(targets[n]),reason=(
            '녹화 관찰 센서의 상대 자세만 조정. 원래 링크 관성·차량 센서·충돌체 보존.' if n==CAM else
            '기존 단독 조향·정지·보호를 보존하고 명시 직선 속도 후보를 연결·기록한다.'))
    chain=(ROOT/CHAIN).read_bytes()
    end=f"'{PARENT}')".encode()
    assert chain.count(end)==1
    before[CHAIN]=sha(chain)
    targets[CHAIN]=chain.replace(end,f"'{PARENT}', '{CHILD}')".encode())
    targets[CHILD]=(json.dumps(contract,ensure_ascii=False,indent=2)+'\n').encode()
    with ZipFile(ARCHIVE,'x',ZIP_DEFLATED) as out:
        for n,b in targets.items():
            out.writestr(n,b)
    manifest=dict(before=before,after={n:sha(b) for n,b in targets.items()},archive_sha256=sha(ARCHIVE.read_bytes()),
                  new_files=[NEW,TEST,CHILD])
    with (HERE/'staged_deployment.json').open('x',encoding='utf8') as f:
        json.dump(manifest,f,indent=2)
    print('prepared',len(targets),'files; no runtime changes')


def install():
    race=HERE.parent/'post_free_race_distance_v1'
    if not (race/'review.exit').exists() or not (race/'runtime.exit').exists():
        raise RuntimeError('registered traffic verification is still open')
    data=json.loads((HERE/'staged_deployment.json').read_bytes())
    assert data['archive_sha256']==sha(ARCHIVE.read_bytes())
    with ZipFile(ARCHIVE) as z:
        targets={n:z.read(n) for n in z.namelist()}
    assert {n:sha(b) for n,b in targets.items()}==data['after']
    assert not any((ROOT/n).exists() for n in data['new_files'])
    for n,h in data['before'].items():
        if sha((ROOT/n).read_bytes())!=h:
            raise ValueError('another change since staging: '+n)
    for n,b in targets.items():
        (ROOT/n).write_bytes(b)
    assert all(sha((ROOT/n).read_bytes())==s for n,s in data['after'].items())
    print('installed',len(targets),'registered files')


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('operation',choices=('prepare','install'))
    (prepare if p.parse_args().operation=='prepare' else install)()
