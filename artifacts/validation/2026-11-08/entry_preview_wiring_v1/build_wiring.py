"""정지차 짧은 목표점 후보의 ROS 옵션 연결 사본. src는 읽기만 한다. 적용 전 before 해시 대조 필수."""
import ast,hashlib,json
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
NODE='src/arena_autonomy/arena_autonomy/g1_g2_node.py';LAUNCH='src/arena_bringup/launch/simulation.launch.py'
staged=A/'staged';before={};after={};newline={}
def read(rel):
    b=(ROOT/rel).read_bytes();before[rel]=hashlib.sha256(b).hexdigest()
    newline[rel]='\r\n' if b'\r\n' in b else '\n'
    return b.decode('utf-8').replace('\r\n','\n')
def change(text,old,new):
    assert text.count(old)==1,(old[:90],text.count(old))
    return text.replace(old,new,1)
def save(rel,text):
    ast.parse(text);data=text.replace('\n',newline[rel]).encode('utf-8')
    path=staged/rel;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
    after[rel]=hashlib.sha256(data).hexdigest()

# 노드: 기존 g1_g2_stopped_entry_centering과 같은 위치·형식으로 선언하고 실행 설정에 전달한다.
t=read(NODE)
t=change(t,"        center_stopped = self.declare_parameter('g1_g2_stopped_entry_centering',False).value\n",
         "        center_stopped = self.declare_parameter('g1_g2_stopped_entry_centering',False).value\n"
         "        adaptive_preview = self.declare_parameter('g1_g2_adaptive_entry_preview',False).value\n")
t=change(t,"stopped_entry_centering=center_stopped,\n",
         "stopped_entry_centering=center_stopped,\n            adaptive_entry_preview=adaptive_preview,\n")
save(NODE,t)

# launch: 기본 false 인자, 명시 우회 시험에서만 허용, 같은 controller_parameters 블록으로 전달한다.
t=read(LAUNCH)
t=change(t,'        center_stopped = LaunchConfiguration("evaluation_stopped_entry_centering").perform(context) == "true"\n',
         '        center_stopped = LaunchConfiguration("evaluation_stopped_entry_centering").perform(context) == "true"\n'
         '        adaptive_preview = LaunchConfiguration("evaluation_adaptive_entry_preview").perform(context) == "true"\n'
         '        if adaptive_preview and reserve == "off":\n'
         '            raise ValueError("adaptive entry preview requires an explicit corridor handoff trial")\n')
t=change(t,'g1_g2_stopped_entry_centering=center_stopped,\n',
         'g1_g2_stopped_entry_centering=center_stopped,\n                g1_g2_adaptive_entry_preview=adaptive_preview,\n')
t=change(t,'            DeclareLaunchArgument("evaluation_stopped_entry_centering", default_value="false", choices=["false","true"]),\n',
         '            DeclareLaunchArgument("evaluation_stopped_entry_centering", default_value="false", choices=["false","true"]),\n'
         '            DeclareLaunchArgument("evaluation_adaptive_entry_preview", default_value="false", choices=["false","true"],\n'
         '                                  description="정지차 우회 경로가 점유 원호로만 막힐 때 짧은 목표점과 정지 정렬을 비교하는 시험 후보"),\n')
save(LAUNCH,t)

overlay=json.loads((ROOT/'artifacts/validation/2026-11-08/entry_preview_probe_v1/overlay_manifest.json').read_text(encoding='utf-8'))
(A/'wiring_manifest.json').write_text(json.dumps(dict(
    scope='staged wiring copies only; src/install/runtime not changed',line_endings=newline,before=before,after=after,
    candidate_overlay=dict(manifest='artifacts/validation/2026-11-08/entry_preview_probe_v1/overlay_manifest.json',
                           before=overlay['before'],after=overlay['after'])),indent=2)+'\n',encoding='utf-8')
print('staged',list(after))
