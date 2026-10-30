"""G2 통로 후보의 명시 연결 편집. 바이트 대조로 한 번만 적용하며 CRLF를 보존한다."""
from pathlib import Path
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
TRIAL='src/arena_autonomy/arena_autonomy/bypass_entry_trial.py'
FOLLOW='src/arena_autonomy/arena_autonomy/bypass_entry_follow.py'
NODE='src/arena_autonomy/arena_autonomy/g2_entry_node.py'


def edit(path,pairs):
    target=ROOT/path
    raw=target.read_bytes().decode('utf-8')
    if '\r\n' not in raw:
        raise ValueError('expected CRLF source: '+path)
    text=raw.replace('\r\n','\n')
    for old,new,count in pairs:
        if text.count(old)!=count:
            raise ValueError(f'{path}: {old[:60]} {text.count(old)}')
        text=text.replace(old,new)
    target.write_bytes(text.replace('\n','\r\n').encode('utf-8'))


snapshot=BASE/'before_connection.zip'
with zipfile.ZipFile(snapshot,'x',zipfile.ZIP_DEFLATED) as archive:
    for path in (TRIAL,FOLLOW,NODE):
        archive.write(ROOT/path,path)

edit(TRIAL,[
 ("""def _compare_entries(scene, now, speed, previous_steering, dt, *, config=None, entry=None, guide=None,
                     stationary_boundaries=False, padding_growth_s=0.):
""","""def _compare_entries(scene, now, speed, previous_steering, dt, *, config=None, entry=None, guide=None,
                     stationary_boundaries=False, padding_growth_s=0., padding_provider=None):
""",1),
 ("""    config, entry = config or FollowConfig(), entry or EntryConfig()
    # 명령 상한과 측정 입력 범위를 구분한다.""","""    config, entry = config or FollowConfig(), entry or EntryConfig()
    # 기본은 기존 함수다. 명시 시험 후보만 같은 호출 형식의 공급자를 넘긴다.
    outer_padding = current_outer_wall_padding if padding_provider is None else padding_provider
    # 명령 상한과 측정 입력 범위를 구분한다.""",1),
 ("padding,classification=current_outer_wall_padding(scene,config,side,evidence['walls'][side],padding_growth_s)",
  "padding,classification=outer_padding(scene,config,side,evidence['walls'][side],padding_growth_s)",1),
 ("padding_by_side[wall_name]=current_outer_wall_padding(scene,config,wall_name,wall,padding_growth_s)",
  "padding_by_side[wall_name]=outer_padding(scene,config,wall_name,wall,padding_growth_s)",1),
 ("""        self.tracked_wall_fit=tracked_wall_fit
        self.reset()
""","""        self.tracked_wall_fit=tracked_wall_fit
        self.padding_provider=None
        self.reset()
""",1),
 ("""                config=self.config,entry=self.entry,stationary_boundaries=self.stationary_outer_boundary))
""","""                config=self.config,entry=self.entry,stationary_boundaries=self.stationary_outer_boundary,
                padding_provider=self.padding_provider))
""",1),
 ("""                stationary_outer_boundary=self.stationary_outer_boundary and self.wall_confirmations>=2))
""","""                stationary_outer_boundary=self.stationary_outer_boundary and self.wall_confirmations>=2),
            padding_provider=self.padding_provider)
""",1),
])
edit(FOLLOW,[
 ("""from arena_autonomy.local_follow import FollowConfig
""","""from arena_autonomy.local_follow import FollowConfig

CORRIDOR_POLICIES = ('off','corridor_v1')
""",1),
 ("""    prefer_earlier_transition: bool = False

    def __post_init__(self):
        if any(type(value) is not bool for value in vars(self).values()):
            raise ValueError('explicit boolean G2 execution options required')
""","""    prefer_earlier_transition: bool = False
    corridor_policy: str = 'off'

    def __post_init__(self):
        if any(type(value) is not bool for key,value in vars(self).items() if key!='corridor_policy'):
            raise ValueError('explicit boolean G2 execution options required')
        if self.corridor_policy not in CORRIDOR_POLICIES:
            raise ValueError('registered G2 corridor policy required')
""",1),
 ("""        self.guide=EntryGuide(self.config,EntryConfig(offsets_m=offsets,
            prefer_earlier_transition=self.execution.prefer_earlier_transition),
            stationary_outer_boundary=self.execution.stationary_outer_boundary,
            wall_reanchor=self.execution.wall_reanchor,tracked_wall_fit=self.execution.tracked_wall_fit,
            rejoin_enabled=self.execution.rejoin_enabled)
""","""        guide_type,options=EntryGuide,{}
        if self.execution.corridor_policy!='off':
            from arena_autonomy.bypass_corridor_trial import CorridorGuide,REGISTERED_POLICIES
            guide_type=CorridorGuide
            options['corridor_policy']=REGISTERED_POLICIES[self.execution.corridor_policy]
        self.guide=guide_type(self.config,EntryConfig(offsets_m=offsets,
            prefer_earlier_transition=self.execution.prefer_earlier_transition),
            stationary_outer_boundary=self.execution.stationary_outer_boundary,
            wall_reanchor=self.execution.wall_reanchor,tracked_wall_fit=self.execution.tracked_wall_fit,
            rejoin_enabled=self.execution.rejoin_enabled,**options)
""",1),
])
edit(NODE,[
 ("""        trace_path=str(self.declare_parameter('g2_trace_path','').value)
""","""        trace_path=str(self.declare_parameter('g2_trace_path','').value)
        corridor=self.declare_parameter('g2_corridor_policy','off').value
        if type(corridor) is not str or corridor not in CORRIDOR_POLICIES:
            raise ValueError('registered G2 corridor policy required')
""",1),
 ("""from arena_autonomy.bypass_entry_follow import BypassEntryFollow,BypassExecutionConfig
""","""from arena_autonomy.bypass_entry_follow import BypassEntryFollow,BypassExecutionConfig,CORRIDOR_POLICIES
""",1),
 ("""        self.follow=BypassEntryFollow(self.follow.config,BypassExecutionConfig(),allow_simulation_trial=True)
""","""        self.follow=BypassEntryFollow(self.follow.config,BypassExecutionConfig(corridor_policy=corridor),
            allow_simulation_trial=True)
""",1),
 ("""        self.get_logger().info('G2 explicit sensor entry trial; publishes /drive through independent safety')
""","""        self.get_logger().info('G2 explicit sensor entry trial; publishes /drive through independent safety')
        if corridor!='off':
            self.get_logger().info('G2 corridor test policy '+corridor+' (simulation assumption, not race default)')
""",1),
])
print('applied')
