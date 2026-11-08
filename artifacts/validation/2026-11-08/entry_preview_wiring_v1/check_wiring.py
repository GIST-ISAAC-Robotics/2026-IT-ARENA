"""연결 사본의 기본 false·양끝 이름·config 전달을 AST로, 실행 설정은 overlay import로 확인한다. src는 읽기만 한다."""
import ast,difflib,hashlib,json,sys
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
PROBE=ROOT/'artifacts/validation/2026-11-08/entry_preview_probe_v1'
NODE='src/arena_autonomy/arena_autonomy/g1_g2_node.py';LAUNCH='src/arena_bringup/launch/simulation.launch.py'
PARAM='g1_g2_adaptive_entry_preview';ARG='evaluation_adaptive_entry_preview';FIELD='adaptive_entry_preview'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=json.loads((A/'wiring_manifest.json').read_text(encoding='utf-8'))
watched=sorted(set(manifest['after'])|set(manifest['candidate_overlay']['after']))
src_before={p:sha(ROOT/p) for p in watched if (ROOT/p).is_file()}
checks={}
def text(p):return p.read_bytes().decode('utf-8').replace('\r\n','\n')
def calls(tree,name):
    return [n for n in ast.walk(tree) if isinstance(n,ast.Call)
            and (getattr(n.func,'id',None)==name or getattr(n.func,'attr',None)==name)]
def keyword(call,name):return next((k.value for k in call.keywords if k.arg==name),None)
def lit(node):
    try:return ast.literal_eval(node)
    except ValueError:return None

# src는 적용 전(before) 또는 GPT 적용 뒤(after)여야 한다. 그 밖의 해시는 새 부모이므로 사본을 다시 만든다.
expected_before={**manifest['candidate_overlay']['before'],**manifest['before']}
expected_after={**manifest['candidate_overlay']['after'],**manifest['after']}
src_state={p:'before' if src_before.get(p)==expected_before.get(p) else 'after' if src_before.get(p)==expected_after[p] else 'other'
           for p in expected_after}
checks['src_is_before_or_after']={p:s!='other' for p,s in src_state.items()}
checks['staged_after_hash']={p:sha(A/'staged'/p)==h for p,h in manifest['after'].items()}
checks['overlay_after_hash']={p:sha(PROBE/'overlay/arena_autonomy'/Path(p).name)==h for p,h in manifest['candidate_overlay']['after'].items()}
# 기존 줄을 바꾸지 않고 추가만 했는지 확인한다.
for rel in (NODE,LAUNCH):
    ops=difflib.SequenceMatcher(None,text(ROOT/rel).splitlines(),text(A/'staged'/rel).splitlines(),autojunk=False).get_opcodes()
    checks['insert_only:'+rel]=all(op in ('equal','insert') for op,*_ in ops)

node=ast.parse(text(A/'staged'/NODE))
declared=[c for c in calls(node,'declare_parameter') if c.args and lit(c.args[0])==PARAM]
assigned=[n.targets[0].id for n in ast.walk(node) if isinstance(n,ast.Assign) and isinstance(n.value,ast.Attribute)
          and n.value.value in declared and isinstance(n.targets[0],ast.Name)]
passed=[keyword(c,FIELD) for c in calls(node,'HandoffExecutionConfig') if keyword(c,FIELD) is not None]
checks['node_declares_default_false']=len(declared)==1 and len(declared[0].args)==2 and lit(declared[0].args[1]) is False
checks['node_passes_to_execution_config']=len(assigned)==1 and len(passed)==1 and getattr(passed[0],'id',None)==assigned[0]

launch=ast.parse(text(A/'staged'/LAUNCH))
args=[c for c in calls(launch,'DeclareLaunchArgument') if c.args and lit(c.args[0])==ARG]
checks['launch_argument_default_false']=len(args)==1 and lit(keyword(args[0],'default_value'))=='false' and lit(keyword(args[0],'choices'))==['false','true']
configs=[c for c in calls(launch,'LaunchConfiguration') if c.args and lit(c.args[0])==ARG]
compare=[n for n in ast.walk(launch) if isinstance(n,ast.Assign) and isinstance(n.value,ast.Compare)
         and isinstance(n.value.left,ast.Call) and getattr(n.value.left.func,'value',None) in configs]
local=[n.targets[0].id for n in compare]
checks['launch_reads_true_only']=len(configs)==1 and len(compare)==1 and lit(compare[0].value.comparators[0])=='true'
updates=[keyword(c,PARAM) for c in calls(launch,'update') if keyword(c,PARAM) is not None]
checks['launch_passes_same_parameter_name']=len(updates)==1 and getattr(updates[0],'id',None)==local[0]
guards=[n for n in ast.walk(launch) if isinstance(n,ast.If) and any(getattr(x,'id',None)==local[0] for x in ast.walk(n.test))
        and any(isinstance(x,ast.Raise) for x in n.body)]
checks['launch_rejects_without_handoff_trial']=len(guards)==1
# 같은 블록(reserve != 'off')에서만 g1_g2 매개변수가 생성되는 기존 구조를 유지한다.
blocks=[n for n in ast.walk(launch) if isinstance(n,ast.If) and any(keyword(c,PARAM) is not None for c in calls(n,'update'))
        and any(keyword(c,'g1_g2_stopped_entry_centering') is not None for c in calls(n,'update'))]
checks['launch_same_block_as_centering']=len(blocks)>=1

# 실행 설정은 기존 overlay 사본을 import한다. 설치본/src 모듈이 아님을 확인한다.
sys.path.insert(0,str(PROBE/'overlay'))
from arena_autonomy import bypass_handoff
from arena_autonomy.bypass_handoff import HandoffExecutionConfig
checks['overlay_module_imported']=Path(bypass_handoff.__file__).resolve().parent==(PROBE/'overlay/arena_autonomy').resolve()
checks['execution_default_false']=HandoffExecutionConfig().adaptive_entry_preview is False
try:HandoffExecutionConfig(adaptive_entry_preview=True);checks['execution_rejects_without_corridor']=False
except ValueError:checks['execution_rejects_without_corridor']=True
# r33 launch(reserve=paced, observed/continuous/moving pass/wall witness/centering ON)가 노드에 넘기는 값과 같은 조합.
from arena_autonomy.bypass_entry_follow import BypassExecutionConfig
r33=dict(preparation_policy='observed_entry_space_v1',persistence_window_policy='trim_before_compare_v1',
    follow_entry_space_enabled=True,moving_follow_reserve=True,closing_follow_reserve=True,recent_forward_reserve=False,
    low_motion_reserve=True,moving_pace_floor=True,reacquire_policy='observed_rejoin_v2',entry_policy='corridor_local_v1',
    preparation_hold_policy='bounded_hold_v1',initial_reference_policy='ready_directional_pair_v2',
    motion_pause_policy='bounded_motion_pause_v1',relative_yield_policy='observed_progress_yield_v3',
    entry_motion_gate=True,stopped_entry_centering=True,observed_forward_pace=True,continuous_forward_pace=True,
    moving_pass_trial=True,moving_pass_profile='slow_v1',
    bypass=BypassExecutionConfig(corridor_policy='corridor_v2_signed_hold',steerable_transition=True,rear_wall_witness=True))
checks['r33_config_default_false']=HandoffExecutionConfig(**r33).adaptive_entry_preview is False
try:checks['r36_config_accepts_preview']=HandoffExecutionConfig(**r33,adaptive_entry_preview=True).adaptive_entry_preview is True
except ValueError as error:checks['r36_config_accepts_preview']=repr(error)
try:HandoffExecutionConfig(**r33,adaptive_entry_preview='true');checks['execution_rejects_untyped']=False
except ValueError:checks['execution_rejects_untyped']=True
fields=set(HandoffExecutionConfig.__dataclass_fields__)
checks['field_matches_node_keyword']=FIELD in fields and 'stopped_entry_centering' in fields
checks['src_unchanged_during_check']={p:sha(ROOT/p)==h for p,h in src_before.items()}

flat=lambda v:all(flat(x) for x in v.values()) if isinstance(v,dict) else v is True
result=dict(scope='staged wiring copies + candidate overlay import; no ROS/Gazebo/build/src change',passed=flat(checks),src_state=src_state,checks=checks)
(A/'check_result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,indent=1));raise SystemExit(0 if result['passed'] else 1)
