"""근접 관측 조각 후보의 실행 연결. 기존 실행은 보관하고 원시 실패를 바꾸지 않는다."""
import hashlib, json, zipfile
from pathlib import Path
A=Path(__file__).resolve().parent
R=A.parents[3]
target='src/arena_autonomy/arena_autonomy/bypass_handoff_corridor_trial.py'
module='src/arena_autonomy/arena_autonomy/nearby_face_motion.py'
snapshot=A/'before_r56.zip'
assert not snapshot.exists()
before=(R/target).read_bytes()
with zipfile.ZipFile(snapshot,'w',zipfile.ZIP_DEFLATED) as z:z.writestr(target,before)
s=(A/'near_fragments_candidate.py').read_text(encoding='utf8')
s=s.replace('"""현재 근접 관측의 끊긴 면을 각각 관찰하는 제한 사본. 사각을 잇지 않는다."""', '''"""선택형 우회 중 가까운 관측 조각을 각각 추적한다.

먼 지주가 통로 계획의 대표 면이 되어도 현재 옆차의 관측 창을 잃지 않게 한다.
이미 선택된 rear-wall-witness 우회 후보에서만 사용하며, 고정 물체로 단정하거나
가림 구간을 연결하지 않는다. 기존 5스캔/.4초·접근/잡음 예산·가속 가정을
각 조각에 따로 적용한다. 순간 접근 속도의 물리적 상계는 아니다.
"""''')
s=s.replace('def apply(self,scene,config,side,wall,history,padding,growth_s=0.):','def apply(self,scene,config,side,wall,history,padding,growth_s=0.,*,active=True):\n        if not active:\n            for monitor in self.monitors:monitor.reset()\n            self.key=None\n            return padding,dict(changed=0,reason="inactive_bypass",unknown_filled=False)')
s=s.replace("before=result[selected].copy()", "reduced=np.where(np.isfinite(age)&(age>=0.),reduced,result[selected])\n                before=result[selected].copy()")
(R/module).write_text(s,encoding='utf8',newline='\n')
s=before.decode('utf8').replace('        self.rear_face_motion={}','        self.rear_face_motion={}\n        self.nearby_face_motion={}')
needle="            self.rear_face_motion={side:RearFaceMotion(self.corridor_policy) for side in ('left','right')}"
s=s.replace(needle,needle+"\n            from arena_autonomy.nearby_face_motion import NearbyFragments\n            self.nearby_face_motion={side:NearbyFragments(self.corridor_policy) for side in ('left','right')}")
s=s.replace("            detail['rear_face_motion']=rear", "            detail['rear_face_motion']=rear\n            padding,nearby=self.nearby_face_motion[side].apply(scene,config,side,wall,\n                self.motion_history,padding,growth_s,active=self.path is not None)\n            detail['nearby_face_motion']=nearby")
(R/target).write_bytes(s.encode('utf8'))
t=(A/'test_near_fragments_candidate.py').read_text(encoding='utf8')
t=t.replace('from near_fragments_candidate import NearbyFragments','from arena_autonomy.nearby_face_motion import NearbyFragments')
t=t.replace('from test_near_face_candidate import CFG,WALL',"from arena_autonomy.local_follow import FollowConfig\nCFG=FollowConfig(wall_offset_m=.505)\nWALL=dict(slope=0.,intercept_m=-.275)")
t+='''
def test_inactive_bypass_clears_evidence_and_preserves_padding():
    c=NearbyFragments(CorridorPolicy())
    for t in (.5,.6,.7,.8,.9):evaluate(c,t)
    s=view(1.);padding=np.full(len(s.rays),.13)
    after,e=c.apply(s,CFG,'left',WALL,history(),padding,active=False)
    np.testing.assert_array_equal(after,padding)
    assert e['changed']==0
    after,e=evaluate(c,1.1)
    assert e['changed']==0

def test_base_padding_and_current_occupied_points_are_not_removed():
    c=NearbyFragments(CorridorPolicy())
    for t in (.5,.6,.7,.8,.9):evaluate(c,t)
    s=view(1.);padding=np.full(len(s.rays),.13)
    points=s.occupied_endpoints_xy_m.copy()
    after,e=c.apply(s,CFG,'left',WALL,history(),padding)
    assert np.all(after>=.025+CFG.measurement_padding_m)
    np.testing.assert_array_equal(s.occupied_endpoints_xy_m,points)
    np.testing.assert_array_equal(padding,np.full(len(s.rays),.13))
    assert e['unknown_filled'] is False and e['static_identity_claimed'] is False
'''
(R/'tests/test_nearby_face_motion.py').write_text(t,encoding='utf8',newline='\n')
sha=lambda b:hashlib.sha256(b).hexdigest()
parent='config/tests/junction_divergence_r55_sha256.json'
contract='config/tests/nearby_face_motion_r56_sha256.json'
record=dict(schema_version=1,scope='r56: separate current nearby components during selected rear-wall-witness bypass; no static object claim or free fill',
    parent_contract=parent,parent_contract_sha256=sha((R/parent).read_bytes()),
    before_snapshot=snapshot.relative_to(R).as_posix(),before_snapshot_sha256=sha(snapshot.read_bytes()),
    approved_changes={target:dict(before=sha(before),after=sha((R/target).read_bytes()),reason='Preserve nearby observed face history while distant post is chosen as planning boundary')},
    approved_evaluation_scripts={},new_dependencies={module:sha((R/module).read_bytes())})
(R/contract).write_text(json.dumps(record,indent=2)+'\n',encoding='utf8',newline='\n')
p=R/'tests/test_source_provenance_r0_plus_b3.py'
s=p.read_text(encoding='utf8');needle="M3_SUCCESSORS += ('config/tests/junction_divergence_r55_sha256.json',)"
assert s.count(needle)==1
s=s.replace(needle,needle+"\nM3_SUCCESSORS += ('config/tests/nearby_face_motion_r56_sha256.json',)")
p.write_text(s,encoding='utf8',newline='\r\n')
print('r56 written')
